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

#ifndef RMF_TRAFFIC__SCHEDULE__PARTICIPANT_HPP
#define RMF_TRAFFIC__SCHEDULE__PARTICIPANT_HPP

#include <rmf_traffic/schedule/ParticipantDescription.hpp>
#include <rmf_traffic/schedule/Writer.hpp>
#include <rmf_traffic/schedule/Rectifier.hpp>

#include <rmf_utils/AssignID.hpp>

#include <unordered_set>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Participant
{
public:

  using PlanId = Writer::PlanId;

  /// Set the whole itinerary for the participant. Every route that was
  /// previously in the itinerary will be removed and replaced with these new
  /// routes.
  ///
  /// \param[in] plan
  ///   A unique ID that this plan is associated with
  ///
  /// \param[in] itinerary
  ///   The new itinerary that the participant should reflect in the schedule.
  bool set(PlanId plan, std::vector<Route> itinerary);

  /// Add more routes for the participant. All of the routes currently in the
  /// itinerary will still be in it.
  ///
  /// \param[in] additional_routes
  ///   The new routes to add to the itinerary.
  void extend(std::vector<Route> additional_routes);

  /// Delay the current itinerary.
  ///
  /// \param[in] delay
  ///   The amount of time to push back the relevant waypoints.
  void delay(Duration delay);

  /// The cumulative delay that has built up since the last call to set().
  ///
  /// \note This value will not grow when there are no are itineraries for this
  /// participant.
  //
  // TODO(MXG): Should extend() also reset this value? Currently it does not.
  Duration delay() const;

  /// Notify the schedule that a checkpoint within a plan has been reached
  void reached(PlanId plan, RouteId route, CheckpointId checkpoint);

  /// The last set of checkpoints that were reached for this plan.
  const std::vector<CheckpointId>& reached() const;

  /// Clear all routes from the itinerary.
  void clear();

  /// Get the current itinerary of the participant.
  const Itinerary& itinerary() const;

  /// Get the current itinerary version for this participant.
  //
  // TODO(MXG): This function needs to be unit tested.
  ItineraryVersion version() const;

  /// Get the description of this participant.
  const ParticipantDescription& description() const;

  /// Get the ID that was assigned to this participant.
  ParticipantId id() const;

  using AssignIDPtr = std::shared_ptr<const rmf_utils::AssignID<PlanId>>;

  /// Use this to get a generator that can assign valid new unique plan IDs.
  /// It is okay to generate a plan ID and not use it, as long as any new
  /// call to set(~) uses a plan ID that was generated more recently than the
  /// last one that was passed to set(~).
  const AssignIDPtr& plan_id_assigner() const;

  /// Use this to assign an ID to a plan. This is equivalent to
  /// plan_id_assigner()->assign()
  PlanId assign_plan_id() const;

  /// Get the current plan ID of the participant
  PlanId current_plan_id() const;

  // This class supports moving but not copying
  Participant(Participant&&) = default;
  Participant& operator=(Participant&&) = default;

  /// The destructor will automatically tell the Writer to unregister this
  /// participant.
  ~Participant() = default;

  class Implementation;
  class Debug;
private:
  // The constructor for this class is private. It should only be constructed
  // using make_participant
  Participant();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Make a participant for the schedule.
///
/// \warning This will throw a std::runtime_error if you pass a nullptr writer.
///
/// \param[in] description
///   A descrition of the participant.
///
/// \param[in] writer
///   An interface to use when writing to the schedule.
///
/// \param[in] rectifier_factory
///   A reference to a factory that can produce a rectifier for this
///   Participant. This is useful for distributed schedule systems that have
///   unreliable connections between the Database and the Participant. Passing
///   in a nullptr indicates that there will never be a need for rectification.
///   For example, if the \code{writer} argument refers to a Database instance,
///   then there is no need for a RectifierRequesterFactory.
Participant make_participant(
  ParticipantDescription description,
  std::shared_ptr<Writer> writer,
  std::shared_ptr<RectificationRequesterFactory> rectifier_factory = nullptr);

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__PARTICIPANT_HPP
