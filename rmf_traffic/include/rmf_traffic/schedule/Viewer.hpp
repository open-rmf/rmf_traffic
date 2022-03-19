/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef RMF_TRAFFIC__SCHEDULE__VIEWER_HPP
#define RMF_TRAFFIC__SCHEDULE__VIEWER_HPP

#include <rmf_traffic/detail/bidirectional_iterator.hpp>

#include <rmf_traffic/schedule/Query.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic/schedule/Itinerary.hpp>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/macros.hpp>
#include <rmf_utils/optional.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A pure abstract interface class that allows users to query for itineraries
/// that are in a schedule.
///
/// This class cannot be instantiated directly. To get a Viewer, you must
/// instantiate an rmf_traffic::schedule::Database or an
/// rmf_traffic::schedule::Mirror object.
class Viewer
{
public:

  /// A read-only view of some Trajectories in a Database or Mirror.
  ///
  /// It is undefined behavior to modify a Database or patch a Mirror while
  /// reading Trajectories from this view. The user of this class is responsible
  /// for managing access to reads vs access to writes.
  class View
  {
  public:

    template<typename E, typename I, typename F>
    using base_iterator = rmf_traffic::detail::bidirectional_iterator<E, I, F>;

    // TODO(MXG): Replace this with a PIMPL class
    struct Element
    {
      const ParticipantId participant;
      const PlanId plan_id;
      const RouteId route_id;
      const std::shared_ptr<const Route> route;
      const ParticipantDescription& description;
    };

    class IterImpl;
    using const_iterator = base_iterator<const Element, IterImpl, View>;
    using iterator = const_iterator;

    /// Returns an iterator to the first element of the View
    const_iterator begin() const;

    /// Returns an iterator to the element following the last element of the
    /// View.
    const_iterator end() const;

    /// Returns the number of elements in this View.
    std::size_t size() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Query this Viewer to get a View of the Trajectories inside of it that
  /// match the Query parameters.
  virtual View query(const Query& parameters) const = 0;

  /// Alternative signature for query()
  virtual View query(
    const Query::Spacetime& spacetime,
    const Query::Participants& participants) const = 0;

  // TODO(MXG): Consider providing an iterator-style API to view participant IDs
  // and participant descriptions.

  /// Get the set of active participant IDs.
  virtual const std::unordered_set<ParticipantId>& participant_ids() const = 0;

  /// Get the information of the specified participant if it is available.
  /// If a participant with the specified ID is not registered with the
  /// schedule, then this will return a nullptr.
  virtual std::shared_ptr<const ParticipantDescription> get_participant(
    ParticipantId participant_id) const = 0;

  /// Get the latest version number of this Database.
  virtual Version latest_version() const = 0;

  // Virtual destructor
  virtual ~Viewer() = default;

  // The Debug class is for internal testing use only. Its definition is not
  // visible to downstream users.
  class Debug;
  class Implementation;
};

//==============================================================================
/// A pure abstract interface class that extends Viewer to allow users to
/// explicitly request the itinerary of a specific participant.
///
/// \note This interface class is separate from Viewer because it is not
/// generally needed by the traffic planning or negotiation systems, and the
/// Snapshot class can perform better if it does not need to provide this
/// function.
class ItineraryViewer : public virtual Viewer
{
public:

  /// Get the itinerary of a specific participant if it is available. If a
  /// participant with the specified ID is not registered with the schedule or
  /// has never submitted an itinerary, then this will return a nullopt.
  virtual std::optional<ItineraryView> get_itinerary(
    ParticipantId participant_id) const = 0;

  /// Get the current plan ID of a specific participant if it is available. If
  /// a participant with the specified ID is not registered with the schedule,
  /// then this will return a nullopt.
  virtual std::optional<PlanId> get_current_plan_id(
    ParticipantId participant_id) const = 0;

  /// Get the current progress of a specific participant. If a participant with
  /// the specified ID is not registered with the schedule or has never made
  /// progress, then this will return a nullptr.
  virtual const std::vector<CheckpointId>* get_current_progress(
    ParticipantId participant_id) const = 0;

  /// Get the current known progress of a specific participant along its current
  /// plan. If no progress has been made, this will have a value of 0.
  virtual ProgressVersion get_current_progress_version(
    ParticipantId participant_id) const = 0;

  /// A handle for maintaining a dependency on the progress of an itinerary.
  class DependencySubscription
  {
  public:

    /// The dependency was reached by the participant
    bool reached() const;

    /// The plan of the participant changed before it ever reached the
    /// dependency
    bool deprecated() const;

    /// Equivalent to reached() || deprecated()
    bool finished() const;

    /// Check what dependency this is subscribed to
    Dependency dependency() const;

    // TODO(MXG): Should this class allow the user to change the callbacks for
    // on_reached and on_changed? I'm concerned that could lead to race
    // conditions or confusing behavior.

    class Implementation;
  private:
    DependencySubscription();
    rmf_utils::unique_impl_ptr<Implementation> _pimpl;
  };

  /// Watch a traffic dependency. When a relevant event happens for the
  /// dependency, the on_reached or on_deprecated will be triggered. If the
  /// event had already come to pass before this function is called, then the
  /// relevant callback will be triggered right away, within the scope of this
  /// function.
  ///
  /// Only one of the callbacks will ever be triggered, and it will only be
  /// triggered at most once.
  ///
  /// \param[in] on_reached
  ///   If the dependency is reached, this will be triggered. on_changed will
  ///   never be triggered afterwards.
  ///
  /// \param[in] on_deprecated
  ///   If the plan of the participant changed before it reached this dependency
  ///   then the dependency is deprecated and this callback will be triggered.
  ///   on_reached will never be triggered afterwards.
  ///
  /// \return an object that maintains the dependency for the viewer.
  virtual DependencySubscription watch_dependency(
    Dependency dependency,
    std::function<void()> on_reached,
    std::function<void()> on_deprecated) const = 0;

  // Virtual destructor
  virtual ~ItineraryViewer() = default;
};

} // namespace schedule

namespace detail {

extern template class bidirectional_iterator<
    const schedule::Viewer::View::Element,
    schedule::Viewer::View::IterImpl,
    schedule::Viewer::View
>;

} // namespace detail
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__VIEWER_HPP
