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

#ifndef RMF_TRAFFIC__SCHEDULE__CHANGE_HPP
#define RMF_TRAFFIC__SCHEDULE__CHANGE_HPP

#include <rmf_traffic/schedule/Itinerary.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic/schedule/Version.hpp>

#include <rmf_utils/macros.hpp>

namespace rmf_traffic {
namespace schedule {

using StorageId = uint64_t;

//==============================================================================
/// A class that describes a change within the schedule
class Change
{
public:

  //============================================================================
  /// The API for an Add change
  class Add
  {
  public:

    /// A description of an addition
    struct Item
    {
      /// The ID of the route being added, relative to the plan it belongs to
      RouteId route_id;

      /// The storage ID of the route
      StorageId storage_id;

      /// The information for the route being added
      ConstRoutePtr route;
    };

    /// Add a set of routes
    Add(PlanId plan, std::vector<Item> additions);

    /// A reference to the Trajectory that was inserted.
    const std::vector<Item>& items() const;

    /// The plan ID that these routes are being added for
    PlanId plan_id() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  //============================================================================
  /// The API for a Delay change
  class Delay
  {
  public:

    /// Add a delay
    ///
    /// \param[in] duration
    ///   The duration of that delay.
    Delay(Duration duration);

    /// The duration of the delay.
    Duration duration() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  //============================================================================
  /// A class that describes an erasing change.
  class Erase
  {
  public:

    /// Constructor
    ///
    /// \param[in] id
    ///   The ID of the route that should be erased
    Erase(std::vector<StorageId> ids);

    const std::vector<StorageId>& ids() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  //============================================================================
  /// A class that provides an update on itinerary progression.
  class Progress
  {
  public:

    /// Constructor
    Progress(
      ProgressVersion version,
      std::vector<CheckpointId> checkpoints);

    ProgressVersion version() const;

    const std::vector<CheckpointId>& checkpoints() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  //============================================================================
  /// A class that describes a participant registration.
  class RegisterParticipant
  {
  public:

    /// Constructor
    ///
    /// \param[in] id
    ///   The ID of the participant
    ///
    /// \param[in] description
    ///   The description of the participant
    RegisterParticipant(
      ParticipantId id,
      ParticipantDescription description);

    /// The ID for the participant
    ParticipantId id() const;

    /// The description of the participant
    const ParticipantDescription& description() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  //============================================================================
  /// A class that specifies a participant to unregister.
  class UnregisterParticipant
  {
  public:

    /// Constructor
    ///
    /// \param[in] id
    ///   The ID of the participant that is being unregistered.
    UnregisterParticipant(ParticipantId id);

    /// The ID for the participant
    ParticipantId id() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  //============================================================================
  /// A class that describes update in the participant info
  class UpdateParticipantInfo
  {
  public:
    /// Constructor
    ///
    /// \param[in] id
    ///   The ID of the participant that is being unregistered.
    UpdateParticipantInfo(ParticipantId id, ParticipantDescription desc);

    /// The ID for the participant
    ParticipantId id() const;

    /// Description for participants
    ParticipantDescription description() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  //============================================================================
  /// A class that describes a culling.
  class Cull
  {
  public:

    /// Constructor
    ///
    /// \param[in] time
    ///   The time before which all routes should be culled
    Cull(Time time);

    Time time() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };
};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__CHANGE_HPP
