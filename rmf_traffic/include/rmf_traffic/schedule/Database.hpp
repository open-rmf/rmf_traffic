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

#ifndef RMF_TRAFFIC__SCHEDULE__DATABASE_HPP
#define RMF_TRAFFIC__SCHEDULE__DATABASE_HPP

#include <rmf_traffic/schedule/Inconsistencies.hpp>
#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Patch.hpp>
#include <rmf_traffic/schedule/Writer.hpp>
#include <rmf_traffic/schedule/Snapshot.hpp>

#include <rmf_utils/macros.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A class that maintains a database of scheduled Trajectories. This class is
/// intended to be used only for the canonical RMF traffic schedule database.
///
/// The Viewer API can be queried to find Trajectories that match certain
/// criteria.
///
/// You can also retrieve update patches from a database. To apply those patches
/// to a downstream Viewer, it is strongly advised to use the
/// rmf_traffic::schedule::Mirror class.
class Database : public ItineraryViewer, public Writer, public Snappable
{
public:

  //============================================================================
  // Writer API
  //============================================================================

  // Documentation inherited from Writer
  void set(
    ParticipantId participant,
    PlanId plan,
    const Itinerary& itinerary,
    StorageId storage_base,
    ItineraryVersion version) final;

  // Documentation inherited from Writer
  void extend(
    ParticipantId participant,
    const Itinerary& routes,
    ItineraryVersion version) final;

  // Documentation inherited from Writer
  void delay(
    ParticipantId participant,
    Duration delay,
    ItineraryVersion version) final;

  // Documentation inherited from Writer
  void reached(
    ParticipantId participant,
    PlanId plan,
    const std::vector<CheckpointId>& reached_checkpoints,
    ProgressVersion version) final;

  // Documentation inherited from Writer
  void clear(
    ParticipantId participant,
    ItineraryVersion version) final;

  // Documentation inherited from Writer
  Registration register_participant(
    ParticipantDescription participant_info) final;

  // Documentation inherited
  void update_description(
    ParticipantId participant,
    ParticipantDescription desc) final;

  /// Before calling this function on a Database, you should set the current
  /// time for the database by calling set_current_time(). This will allow the
  /// database to cull this participant after a reasonable amount of time has
  /// passed.
  void unregister_participant(
    ParticipantId participant) final;


  //============================================================================
  // Viewer API
  //============================================================================

  // Documentation inherited from Viewer
  View query(const Query& parameters) const final;

  // Documentation inherited from Viewer
  View query(
    const Query::Spacetime& spacetime,
    const Query::Participants& participants) const final;

  // Documentation inherited from Viewer
  const std::unordered_set<ParticipantId>& participant_ids() const final;

  // Documentation inherited from Viewer
  std::shared_ptr<const ParticipantDescription> get_participant(
    std::size_t participant_id) const final;

  // Documentation inherited from Viewer
  Version latest_version() const;


  //============================================================================
  // ItineraryViewer API
  //============================================================================

  // Documentation inherited from ItineraryViewer
  std::optional<ItineraryView> get_itinerary(
    std::size_t participant_id) const final;

  // Documentation inherited from ItineraryViewer
  std::optional<PlanId> get_current_plan_id(
    std::size_t participant_id) const final;

  // Documentation inherited from ItineraryViewer
  const std::vector<CheckpointId>* get_current_progress(
    ParticipantId participant_id) const final;

  // Documentation inherited from ItineraryViewer
  ProgressVersion get_current_progress_version(
    ParticipantId participant_id) const final;

  // Documentation inherited from ItineraryViewer
  DependencySubscription watch_dependency(
    Dependency dependency,
    std::function<void()> on_reached,
    std::function<void()> on_deprecated) const final;


  //============================================================================
  // Snappable API
  //============================================================================

  // Documentation inherited from Snappable
  std::shared_ptr<const Snapshot> snapshot() const final;

  //============================================================================
  // Database API
  //============================================================================

  /// Initialize a Database
  Database();

  /// A description of all inconsistencies currently present in the database.
  /// Inconsistencies are isolated between Participants.
  ///
  /// To fix the inconsistency, the Participant should resend every Itinerary
  /// change that was missing from every range, or else send a change that
  /// nullifies all previous changes, such as a set(~) or erase(ParticipantId).
  const Inconsistencies& inconsistencies() const;

  /// Get the changes in this Database that match the given Query parameters.
  /// If a version number is specified, then the returned Patch will reflect the
  /// changes that occurred from the specified version to the current version of
  /// the schedule.
  ///
  /// To get a consistent reflection of the schedule when specifying a base
  /// version, it is important that the query parameters are not changed in
  /// between calls.
  ///
  /// \param[in] parameters
  ///   The parameters describing what types of schedule entries the mirror
  ///   cares about.
  ///
  /// \param[in] after
  ///   Specify that only changes which come after this version number are
  ///   desired. If you give a nullopt for this argument, then all changes will
  ///   be provided.
  ///
  /// \return A Patch of schedule changes that are relevant to the specified
  /// query parameters.
  Patch changes(
    const Query& parameters,
    std::optional<Version> after) const;

  /// View the routes that match the parameters and have changed (been added or
  /// delayed) since the specified version. This is useful for viewing
  /// incremental changes.
  ///
  /// \param[in] parameters
  ///   The parameters describing what types of schedule entries are relevant.
  ///
  /// \param[in] after
  ///   Specify that only routes which changed after this version number are
  ///   desired.
  ///
  /// \return a view of the routes that are different since the specified
  /// version.
  View query(
    const Query& parameters,
    Version after) const;

  /// Excessive cumulative delays have a risk of overloading the schedule
  /// database by taking up an excessive amount of memory to track the delay
  /// history. Typically this would be a cumulative delay on the scale of
  /// weeks, months, years, etc, and almost certainly indicates an error in
  /// the schedule participant's reporting.
  ///
  /// Still, to avoid harmful effects of participant errors, when the database
  /// detects an excessive cumulative delay for a participant, a new delay(~)
  /// command will be converted into a set(~) command. This function lets you
  /// decide what the threshold should be for that. The default is equivalent to
  /// 2 hours.
  void set_maximum_cumulative_delay(rmf_traffic::Duration maximum_delay);

  /// Get the current cumulative delay for the participant.
  std::optional<rmf_traffic::Duration> get_cumulative_delay(
    ParticipantId participant) const;

  /// Throw away all itineraries up to the specified time.
  ///
  /// \param[in] time
  ///   All Trajectories that finish before this time will be culled from the
  ///   Database. Their data will be completely deleted from this Database
  ///   object.
  ///
  /// \return The new version of the schedule database. If nothing was culled,
  /// this version number will remain the same.
  Version cull(Time time);

  /// Set the current time on the database. This should be used immediately
  /// before calling unregister_participant() so that the database can cull the
  /// existence of the participant at an appropriate time. There's no need to
  /// call this function for any other purpose.
  void set_current_time(Time time);

  /// Get the current itinerary version for the specified participant.
  //
  // TODO(MXG): This function needs unit testing
  ItineraryVersion itinerary_version(ParticipantId participant) const;

  /// Get the last Plan ID used by this participant.
  ///
  /// This provides the same information as get_current_plan_id, except it
  /// throws an exception instead of returning an optional if the participant
  /// does not exist.
  PlanId latest_plan_id(ParticipantId participant) const;

  /// Get the last Storage ID used by this participant.
  StorageId next_storage_base(ParticipantId participant) const;

  class Implementation;
  class Debug;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace schedule

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__DATABASE_HPP
