/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__TIMELINE_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__TIMELINE_HPP

#include "../DetectConflictInternal.hpp"

#include <rmf_traffic/schedule/Query.hpp>

#include <map>
#include <unordered_map>

namespace rmf_traffic {
namespace schedule {

using StorageId = uint64_t;

namespace {

// TODO(MXG): Consider allowing these values to be configured

// Each Timeline Bucket spans a range of 1 minute.
const Duration BucketDuration = std::chrono::minutes(1);

// This constant is used during the creation of the first bucket for a timeline.
// It's a very minor optimization that avoids making a bucket that will
// potentially not be very useful.
const Duration PartialBucketDuration = std::chrono::seconds(50);

} // anonymous namespace

//==============================================================================
struct ParticipantFilter
{
  static std::unordered_set<ParticipantId> convert(
    const std::vector<ParticipantId>& ids)
  {
    std::unordered_set<ParticipantId> output;
    for (const auto id : ids)
      output.insert(id);

    return output;
  }

  //============================================================================
  struct AllowAll
  {
    bool ignore(ParticipantId) const
    {
      return false;
    }
  };

  //============================================================================
  struct Include
  {
    Include(const std::vector<ParticipantId>& ids)
    : _ids(convert(ids))
    {
      // Do nothing
    }

    bool ignore(ParticipantId id) const
    {
      return _ids.find(id) == _ids.end();
    }

  private:
    std::unordered_set<ParticipantId> _ids;
  };

  //============================================================================
  struct Exclude
  {
    Exclude(const std::vector<ParticipantId>& ids)
    : _ids(convert(ids))
    {
      // Do nothing
    }

    bool ignore(ParticipantId id) const
    {
      return _ids.find(id) != _ids.end();
    }

  private:
    std::unordered_set<ParticipantId> _ids;
  };
};

//==============================================================================
template<typename Entry>
class TimelineInspector;

//==============================================================================
template<typename Entry>
class Timeline;

//==============================================================================
template<typename Entry>
class TimelineView
{
public:
public:

  using ConstEntryPtr = std::shared_ptr<const Entry>;
  using Bucket = std::vector<ConstEntryPtr>;

  // We use a shared_ptr for BucketPtr so that the Handle class can hold a
  // weak_ptr to the bucket that contains its entry. If the bucket is ever
  // deleted (e.g. because of a culling) that won't have a negative impact on
  // the Handle's cleanup.
  using BucketPtr = std::shared_ptr<Bucket>;
  using Checked =
    std::unordered_map<ParticipantId, std::unordered_set<RouteId>>;

  // TODO(MXG): Come up with a better name for this data structure than Entries
  using Entries = std::map<Time, BucketPtr>;
  using MapNameToEntries = std::unordered_map<std::string, Entries>;

  /// Constructor
  TimelineView()
  : _all_bucket(std::make_shared<Bucket>())
  {
    // Do nothing
  }

  /// Inspect the timeline for entries that match the query
  template<typename Inspector>
  void inspect(
    const Query::Spacetime& spacetime,
    const Query::Participants& participants,
    Inspector& inspector) const
  {
    const Query::Participants::Mode mode = participants.get_mode();

    if (Query::Participants::Mode::All == mode)
    {
      inspect_spacetime(
        spacetime,
        ParticipantFilter::AllowAll(),
        inspector);
    }
    else if (Query::Participants::Mode::Include == mode)
    {
      inspect_spacetime(
        spacetime,
        ParticipantFilter::Include(participants.include()->get_ids()),
        inspector);
    }
    else if (Query::Participants::Mode::Exclude == mode)
    {
      inspect_spacetime(
        spacetime,
        ParticipantFilter::Exclude(participants.exclude()->get_ids()),
        inspector);
    }
    else
    {
      // *INDENT-OFF*
      throw std::runtime_error(
        "Unexpected Query::Participants mode: "
        + std::to_string(static_cast<uint16_t>(mode)));
      // *INDENT-ON*
    }
  }

  template<typename> friend class Timeline;

protected:

  template<typename Inspector, typename ParticipantFilter>
  void inspect_spacetime(
    const Query::Spacetime& spacetime,
    const ParticipantFilter& participant_filter,
    Inspector& inspector) const
  {
    const Query::Spacetime::Mode mode = spacetime.get_mode();

    if (Query::Spacetime::Mode::All == mode)
    {
      inspect_all_spacetime(participant_filter, inspector);
    }
    else if (Query::Spacetime::Mode::Regions == mode)
    {
      inspect_spacetime_regions(
        *spacetime.regions(), participant_filter, inspector);
    }
    else if (Query::Spacetime::Mode::Timespan == mode)
    {
      inspect_spacetime_timespan(
        *spacetime.timespan(), participant_filter, inspector);
    }
  }

  template<typename Inspector, typename ParticipantFilter>
  void inspect_all_spacetime(
    const ParticipantFilter& participant_filter,
    Inspector& inspector) const
  {
    Checked checked;

    const auto relevant = [](const Entry&) -> bool { return true; };
    for (const auto& entry : *_all_bucket)
    {
      if (!entry->description)
      {
        // This entry is still missing its participant description, so it
        // should be ignored for now.
        continue;
      }

      if (participant_filter.ignore(entry->participant))
        continue;

      if (!checked[entry->participant].insert(entry->storage_id).second)
        continue;

      inspector.inspect(entry.get(), relevant);
    }
  }

  template<typename Inspector, typename ParticipantFilter>
  void inspect_spacetime_regions(
    const Query::Spacetime::Regions& regions,
    const ParticipantFilter& participant_filter,
    Inspector& inspector) const
  {
    Checked checked;

    rmf_traffic::internal::Spacetime spacetime_data;
    const auto relevant =
      [&spacetime_data](const Entry& entry) -> bool
      {
        return rmf_traffic::internal::detect_conflicts(
          entry.description->profile(),
          entry.route->trajectory(),
          spacetime_data);
      };

    for (const Region& region : regions)
    {
      const std::string& map = region.get_map();
      const auto map_it = _timelines.find(map);
      if (map_it == _timelines.end())
        continue;

      const Entries& timeline = map_it->second;
      const Time* const lower_time_bound = region.get_lower_time_bound();
      const Time* const upper_time_bound = region.get_upper_time_bound();

      spacetime_data.lower_time_bound = lower_time_bound;
      spacetime_data.upper_time_bound = upper_time_bound;

      const auto timeline_begin =
        get_timeline_begin(timeline, lower_time_bound);

      const auto timeline_end =
        get_timeline_end(timeline, upper_time_bound);

      if (timeline_begin == timeline_end)
      {
        // No buckets in the timeline overlap with this spacetime region, so we
        // can move on to the next region without looping through this one.
        continue;
      }

      for (auto space_it = region.begin(); space_it != region.end(); ++space_it)
      {
        spacetime_data.pose = space_it->get_pose();
        spacetime_data.shape = space_it->get_shape();

        inspect_entries(
          relevant,
          participant_filter,
          inspector,
          timeline_begin,
          timeline_end,
          checked);
      }
    }
  }

  template<typename Inspector, typename ParticipantFilter>
  void inspect_spacetime_timespan(
    const Query::Spacetime::Timespan& timespan,
    const ParticipantFilter& participant_filter,
    Inspector& inspector) const
  {
    Checked checked;

    const Time* const lower_time_bound = timespan.get_lower_time_bound();
    const Time* const upper_time_bound = timespan.get_upper_time_bound();

    const auto relevant = [&lower_time_bound, &upper_time_bound](
      const Entry& entry) -> bool
      {
        const Trajectory& trajectory = entry.route->trajectory();
        assert(trajectory.start_time());
        if (lower_time_bound && *trajectory.finish_time() < *lower_time_bound)
          return false;

        if (upper_time_bound && *upper_time_bound < *trajectory.start_time())
          return false;

        return true;
      };

    if (timespan.all_maps())
    {
      for (const auto& timeline_it : _timelines)
      {
        const Entries& timeline = timeline_it.second;
        inspect_entries(
          relevant,
          participant_filter,
          inspector,
          get_timeline_begin(timeline, lower_time_bound),
          get_timeline_end(timeline, upper_time_bound),
          checked);
      }
    }
    else
    {
      const auto& maps = timespan.maps();
      for (const std::string& map : maps)
      {
        const auto map_it = _timelines.find(map);
        if (map_it == _timelines.end())
          continue;

        const Entries& timeline = map_it->second;
        inspect_entries(
          relevant,
          participant_filter,
          inspector,
          get_timeline_begin(timeline, lower_time_bound),
          get_timeline_end(timeline, upper_time_bound),
          checked);
      }
    }
  }

  template<typename Inspector, typename ParticipantFilter>
  void inspect_entries(
    const std::function<bool(const Entry&)>& relevant,
    const ParticipantFilter& participant_filter,
    Inspector& inspector,
    const typename Entries::const_iterator& timeline_begin,
    const typename Entries::const_iterator& timeline_end,
    Checked& checked) const
  {
    auto timeline_it = timeline_begin;
    for (; timeline_it != timeline_end; ++timeline_it)
    {
      const Bucket& bucket = *timeline_it->second;

      auto entry_it = bucket.begin();
      for (; entry_it != bucket.end(); ++entry_it)
      {
        const Entry* entry = entry_it->get();

        if (!entry->description)
        {
          // This entry is still missing its participant description, so it
          // should be ignored for now.
          continue;
        }

        if (participant_filter.ignore(entry->participant))
          continue;

        if (!checked[entry->participant].insert(entry->storage_id).second)
          continue;

        inspector.inspect(entry, relevant);
      }
    }
  }

  static typename Entries::const_iterator get_timeline_begin(
    const Entries& timeline,
    const Time* const lower_time_bound)
  {
    return (lower_time_bound == nullptr) ?
      timeline.begin() : timeline.lower_bound(*lower_time_bound);
  }

  static typename Entries::const_iterator get_timeline_end(
    const Entries& timeline,
    const Time* upper_time_bound)
  {
    if (upper_time_bound == nullptr)
      return timeline.end();

    auto end = timeline.upper_bound(*upper_time_bound);
    if (end == timeline.end())
      return end;

    return ++end;
  }

  MapNameToEntries _timelines;
  BucketPtr _all_bucket;
};

//==============================================================================
struct BaseRouteEntry
{
  ConstRoutePtr route;
  ParticipantId participant;
  PlanId plan_id;
  RouteId route_id;
  StorageId storage_id;

  std::shared_ptr<const ParticipantDescription> description;
};

//==============================================================================
template<typename Entry>
class Timeline : public TimelineView<Entry>
{
public:

  using ConstEntryPtr = typename TimelineView<Entry>::ConstEntryPtr;
  using Bucket = typename TimelineView<Entry>::Bucket;
  using BucketPtr = typename TimelineView<Entry>::BucketPtr;
  using Entries = typename TimelineView<Entry>::Entries;

  using BaseBucket = TimelineView<BaseRouteEntry>::Bucket;
  using BaseBucketPtr = TimelineView<BaseRouteEntry>::BucketPtr;

  static BaseBucketPtr clone_bucket(
    const Bucket& other,
    const std::function<bool(const Entry& other)>& check_relevant)
  {
    if constexpr (std::is_same<Entry, BaseRouteEntry>::value)
    {
      // If there is no need to check for relevance and we are using
      if (!check_relevant)
        return std::make_shared<BaseBucket>(other);
    }

    BaseBucket output;
    output.reserve(other.size());

    for (const auto& other_entry : other)
    {
      assert(other_entry);
      if (check_relevant)
      {
        if (!check_relevant(*other_entry))
          continue;
      }

      output.emplace_back(std::make_shared<BaseRouteEntry>(*other_entry));
    }

    return std::make_shared<BaseBucket>(std::move(output));
  }

  /// This Timeline::Handle class allows us to use RAII so that when an Entry is
  /// deleted it will automatically be removed from any of its timeline buckets.
  struct Handle
  {
    Handle(
      ConstEntryPtr entry,
      std::vector<std::weak_ptr<Bucket>> buckets)
    : _entry(std::move(entry)),
      _buckets(std::move(buckets))
    {
      // Do nothing
    }

    ~Handle()
    {
      for (const auto& b : _buckets)
      {
        BucketPtr bucket = b.lock();
        if (!bucket)
          continue;

        const auto it = std::find(bucket->begin(), bucket->end(), _entry);
        if (it != bucket->end())
          bucket->erase(it);
      }
    }

  private:
    ConstEntryPtr _entry;
    std::vector<std::weak_ptr<Bucket>> _buckets;
  };

  /// Insert a new entry into the timeline
  std::shared_ptr<Handle> insert(
    const std::shared_ptr<Entry>& entry)
  {
    if (!entry)
    {
      // *INDENT-OFF*
      throw std::runtime_error(
        "[rmf_traffic::schedule::Timeline::insert] INTERNAL ERROR: "
        "nullptr value for entry being inserted. Please report this bug to the "
        "maintainers!");
      // *INDENT-ON*
    }

    std::vector<std::weak_ptr<Bucket>> buckets;
    this->_all_bucket->push_back(entry);
    buckets.emplace_back(this->_all_bucket);

    if (entry->route && entry->route->trajectory().size() < 2)
    {
      // *INDENT-OFF*
      throw std::runtime_error(
        "[rmf_traffic::schedule::Timeline] Trying to insert a trajectory with "
        "less than 2 waypoints ["
        + std::to_string(entry->route->trajectory().size()) + "] is illegal!");
      // *INDENT-ON*
    }

    if (entry->route && entry->route->trajectory().start_time())
    {

      const Time start_time = *entry->route->trajectory().start_time();
      const Time finish_time = *entry->route->trajectory().finish_time();
      const std::string& map_name = entry->route->map();

      const auto map_it = this->_timelines.insert(
        std::make_pair(map_name, Entries())).first;

      Entries& timeline = map_it->second;

      const auto start_it = get_timeline_iterator(timeline, start_time);
      const auto end_it = ++get_timeline_iterator(timeline, finish_time);

      for (auto it = start_it; it != end_it; ++it)
      {
        it->second->push_back(entry);
        buckets.emplace_back(it->second);
      }
    }

    return std::make_shared<Handle>(entry, std::move(buckets));
  }

  void cull(const Time time)
  {
    for (auto& pair : this->_timelines)
    {
      auto& timeline = pair.second;

      // NOTE(MXG): It is not an error that we are using get_timeline_begin() to
      // find the ending iterator. We want to stop just before the first bucket
      // that contains the cull time, because we only want to erase times that
      // come before it.
      const auto end_it =
        TimelineView<Entry>::get_timeline_begin(timeline, &time);

      if (end_it != timeline.begin())
        timeline.erase(timeline.begin(), end_it);
    }
  }

  /// Create an immutable snapshot of the current timeline. A single instance of
  /// the snapshot can be safely used by multiple threads simultaneously.
  ///
  /// Filter out entries based on which are relevant.
  std::shared_ptr<const TimelineView<const BaseRouteEntry>> snapshot(
    const std::function<bool(const Entry& other)>& check_relevant) const
  {
    // TODO(MXG): Consider whether we could use plain std::vector<Bucket>
    // instances instead of std::vector<std::shared_ptr<Bucket>> in the snapshot
    // to reduce how many heap allocations are needed, and to lessen the cache
    // misses. The tricky part is this may cost us more when copying the maps.

    // TODO(MXG): Consider whether there's a safe way to cache a snapshot. The
    // main problem I see is that we don't have a good way of knowing when an
    // entry gets removed right now, so we wouldn't know to update the snapshot
    // when an entry gets taken out.

    std::shared_ptr<TimelineView<const BaseRouteEntry>> result =
      std::make_shared<TimelineView<const BaseRouteEntry>>();

    // Make a deep copy of the buckets so that adding/removing entries from the
    // source bucket does not impact the snapshot
    if constexpr (std::is_same<Entry, BaseRouteEntry>::value)
    {
      result->_timelines = this->_timelines;
      for (auto& [map, time_scope] : result->_timelines)
      {
        for (auto& [time, bucket] : time_scope)
          bucket = clone_bucket(*bucket, check_relevant);
      }
    }
    else
    {
      result->_timelines.reserve(this->_timelines.size());
      for (const auto& [map, time_scope] : this->_timelines)
      {
        auto& out_time_scope =
          result->_timelines.insert({map, {}}).first->second;

        for (const auto& [time, bucket] : time_scope)
          out_time_scope.insert({time, clone_bucket(*bucket, check_relevant)});
      }
    }

    result->_all_bucket = clone_bucket(*this->_all_bucket, check_relevant);

    return result;
  }

private:

  //============================================================================
  static typename Entries::iterator get_timeline_iterator(
    Entries& timeline, const Time time)
  {
    auto start_it = timeline.lower_bound(time);

    if (start_it == timeline.end())
    {
      if (timeline.empty())
      {
        // This timeline is completely empty, so we'll begin creating buckets
        // starting from the time of this trajectory.
        return timeline.insert(
          timeline.end(),
          std::make_pair(
            time + PartialBucketDuration,
            std::make_shared<Bucket>()));
      }

      auto last_it = --timeline.end();
      while (last_it->first < time)
      {
        last_it = timeline.insert(
          timeline.end(),
          std::make_pair(
            last_it->first + BucketDuration,
            std::make_shared<Bucket>()));
      }

      return last_it;
    }

    while (time + BucketDuration < start_it->first)
    {
      start_it = timeline.insert(
        start_it,
        std::make_pair(
          start_it->first - BucketDuration,
          std::make_shared<Bucket>()));
    }

    return start_it;
  }
};

//==============================================================================
template<typename Entry>
class TimelineInspector
{
public:

  virtual void inspect(
    const Entry* entry,
    const std::function<bool(const Entry& entry)>& relevant) = 0;

  virtual ~TimelineInspector() = default;
};


} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__TIMELINE_HPP
