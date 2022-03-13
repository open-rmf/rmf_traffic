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

#ifndef RMF_TRAFFIC__SCHEDULE__MIRROR_HPP
#define RMF_TRAFFIC__SCHEDULE__MIRROR_HPP

#include <rmf_traffic/schedule/Database.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A class that maintains a mirror of a Database of scheduled Trajectories.
/// This class is intended to provide a cache of the scheduled Trajectories to
/// processes or threads that do not contain the original upstream copy of the
/// rmf_traffic::schedule::Database.
///
/// The Mirror is designed to mirror a relevant subset of the schedule database.
class Mirror : public ItineraryViewer, public Snappable
{
public:

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
  std::optional<ItineraryView> get_itinerary(
    std::size_t participant_id) const final;

  // Documentation inherited from Viewer
  Version latest_version() const final;


  //============================================================================
  // ItineraryViewer API
  //============================================================================

  // Documentation inherited from ItineraryViewer
  std::optional<PlanId> get_current_plan_id(
    ParticipantId participant_id) const final;

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
  std::shared_ptr<const Snapshot> snapshot() const final;


  //============================================================================
  // Mirror API
  //============================================================================

  /// Create a database mirror
  Mirror();

  /// Update the known participants and their descriptions.
  void update_participants_info(const ParticipantDescriptionsMap& participants);

  /// Update this mirror.
  ///
  /// \return true if this update is okay. false if the base version of the
  /// patch does not match
  bool update(const Patch& patch);

  /// Fork a new database off of this Mirror. The state of the new database
  /// will match the last state of the upstream database that this Mirror knows
  /// about.
  Database fork() const;

  // TODO(MXG): Consider a feature to log and report any possible
  // inconsistencies that might show up with the patches, e.g. replacing or
  // erasing a trajectory that was never received in the first place.

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};


} // namespace schedule
} // namepsace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__MIRROR_HPP
