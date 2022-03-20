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

#ifndef RMF_TRAFFIC__SCHEDULE__WRITER_HPP
#define RMF_TRAFFIC__SCHEDULE__WRITER_HPP

#include <rmf_traffic/schedule/Itinerary.hpp>
#include <rmf_traffic/schedule/ParticipantDescription.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A pure abstract interface class that defines an API for writing to the
/// schedule database. This API is implemented by the Database class, but it
/// should also be implemented for any middleware that intends to have a
/// schedule participant write changes to a remote database.
class Writer
{
public:

  using ParticipantId = rmf_traffic::schedule::ParticipantId;
  using ParticipantDescription = rmf_traffic::schedule::ParticipantDescription;
  using Itinerary = rmf_traffic::schedule::Itinerary;
  using ItineraryVersion = rmf_traffic::schedule::ItineraryVersion;
  using ProgressVersion = rmf_traffic::schedule::ProgressVersion;
  using PlanId = rmf_traffic::PlanId;
  using Duration = rmf_traffic::Duration;
  using RouteId = rmf_traffic::RouteId;
  using CheckpointId = rmf_traffic::CheckpointId;
  using StorageId = uint64_t;

  /// Set a brand new itinerary for a participant. This will replace any
  /// itinerary that is already in the schedule for the participant.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being updated.
  ///
  /// \param[in] plan
  ///   The ID of the plan that this new itinerary belongs to.
  ///
  /// \param[in] itinerary
  ///   The new itinerary of the participant.
  ///
  /// \param[in] storage_base
  ///   The storage index offset that the database should use for this plan.
  ///   This should generally be the integer number of total routes that the
  ///   participant has ever given to the writer prior to setting this new
  ///   itinerary. This value helps ensure consistent unique IDs for every
  ///   route, even after a database has failed over or restarted.
  ///
  /// \param[in] version
  ///   The version for this itinerary change.
  virtual void set(
    ParticipantId participant,
    PlanId plan,
    const Itinerary& itinerary,
    StorageId storage_base,
    ItineraryVersion version) = 0;

  /// Add a set of routes to the itinerary of this participant.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being updated.
  ///
  /// \param[in] routes
  ///   The set of routes that should be added to the itinerary.
  ///
  /// \param[in] version
  ///   The version for this itinerary change
  ///
  virtual void extend(
    ParticipantId participant,
    const Itinerary& routes,
    ItineraryVersion version) = 0;

  /// Add a delay to the itinerary from the specified Time.
  ///
  /// Nothing about the routes in the itinerary will be changed except that
  /// waypoints will shifted through time.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being delayed.
  ///
  /// \param[in] delay
  ///   This is the duration of time to delay all qualifying Trajectory
  ///   Waypoints.
  ///
  /// \param[in] version
  ///   The version for this itinerary change
  ///
  virtual void delay(
    ParticipantId participant,
    Duration delay,
    ItineraryVersion version) = 0;

  /// Indicate that a participant has reached certain checkpoints.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose progress is being set.
  ///
  /// \param[in] plan
  ///   The ID of the plan which progress has been made for.
  ///
  /// \param[in] reached_checkpoints
  ///   The set of checkpoints that have been reached. The indices in the vector
  ///   must correspond to the RouteIds of the plan.
  ///
  /// \param[in] version
  ///   The version number for this progress.
  ///
  virtual void reached(
    ParticipantId participant,
    PlanId plan,
    const std::vector<CheckpointId>& reached_checkpoints,
    ProgressVersion version) = 0;

  /// Erase an itinerary from this database.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being erased.
  ///
  /// \param[in] version
  ///   The version for this itinerary change
  ///
  virtual void clear(
    ParticipantId participant,
    ItineraryVersion version) = 0;

  /// Information resulting from registering a participant
  class Registration
  {
  public:

    /// Constructor
    ///
    /// \param[in] id
    ///   The ID for the registered participant
    ///
    /// \param[in] version
    ///   The last itinerary version for the registered participant
    ///
    /// \param[in] plan_id
    ///   The last plan_id for the registered participant
    ///
    /// \param[in] storage_base
    ///   The next storage base that the registered participant should use
    Registration(
      ParticipantId id,
      ItineraryVersion version,
      PlanId plan_id,
      StorageId storage_base);

    /// The ID of the registered participant
    ParticipantId id() const;

    /// The last itinerary version of the registered participant. New
    /// Participants will begin by adding up from this version when issuing
    /// schedule updates.
    ///
    /// This value might vary for systems that enforce participant uniqueness.
    /// If this participant was registered in the past and is now being
    /// re-registered, then the version number will pick up where it previously
    /// left off.
    ItineraryVersion last_itinerary_version() const;

    /// The last Route ID of the registered participant. New Participants will
    /// begin by adding up from this Route ID when issuing new schedule updates.
    ///
    /// Similar to last_itinerary_version, this value might vary for systems
    /// that enforce participant uniqueness.
    PlanId last_plan_id() const;

    /// The next storage base that the participant should use.
    StorageId next_storage_base() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Register a new participant.
  ///
  /// \param[in] participant_info
  ///   Information about the new participant.
  ///
  /// \param[in] time
  ///   The time at which the registration is being requested.
  ///
  /// \return result of registering the new participant.
  virtual Registration register_participant(
    ParticipantDescription participant_info) = 0;

  /// Unregister an existing participant.
  ///
  /// \param[in] participant
  ///   The ID of the participant to unregister.
  ///
  /// \return the new version of the schedule.
  virtual void unregister_participant(
    ParticipantId participant) = 0;

  /// Updates a participants footprint
  ///
  /// \param[in] participant
  ///   The ID of the participant to update
  /// \param[in] desc
  ///   The participant description
  virtual void update_description(
    ParticipantId participant,
    ParticipantDescription desc) = 0;

  // virtual destructor
  virtual ~Writer() = default;

};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__WRITER_HPP
