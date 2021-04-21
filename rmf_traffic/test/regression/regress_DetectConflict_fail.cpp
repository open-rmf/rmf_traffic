/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <rmf_utils/catch.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/DetectConflict.hpp>

#include <fstream>

namespace {

class SerializedWaypoint
{
public:
  Eigen::Vector3d position, velocity;

  rmf_traffic::Time time;
};

SerializedWaypoint read_waypoint(const std::string& file_name)
{
  std::ifstream file;
  file.open(file_name, std::ios::in);
  SerializedWaypoint serialized_waypoint;
  file.seekg(0);
  file.read((char*)&serialized_waypoint, sizeof(SerializedWaypoint));
  file.close();
  return serialized_waypoint;
}

void write_waypoint(const std::string& file_name,
  SerializedWaypoint& serialized_waypoint)
{
  std::ofstream file;
  file.open(file_name, std::ios::out);
  file.write((char*) &serialized_waypoint, sizeof(SerializedWaypoint));
  file.close();
}

//==============================================================================
SCENARIO("Failed Detect Conflict")
{
  SerializedWaypoint serialized_waypoint[4];
  serialized_waypoint[0] =
    read_waypoint(RESOURCES_DIR + std::string("/Waypoint5.txt"));
  serialized_waypoint[1] =
    read_waypoint(RESOURCES_DIR + std::string("/Waypoint6.txt"));
  serialized_waypoint[2] =
    read_waypoint(RESOURCES_DIR + std::string("/Waypoint7.txt"));
  serialized_waypoint[3] =
    read_waypoint(RESOURCES_DIR + std::string("/Waypoint8.txt"));

  rmf_traffic::agv::VehicleTraits traits{
    {0.5, 2.0}, {0.75, 1.5},
    rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex(
        rmf_traffic::geometry::Circle(0.2))
    }
  };

  rmf_traffic::Trajectory traj1, traj2;
  traj1.insert(
    serialized_waypoint[0].time,
    serialized_waypoint[0].position,
    serialized_waypoint[0].velocity);
  traj1.insert(
    serialized_waypoint[1].time,
    serialized_waypoint[1].position,
    serialized_waypoint[1].velocity);

  traj2.insert(
    serialized_waypoint[2].time,
    serialized_waypoint[2].position,
    serialized_waypoint[2].velocity);
  traj2.insert(
    serialized_waypoint[3].time,
    serialized_waypoint[3].position,
    serialized_waypoint[3].velocity);

  bool detect_conflict =
    rmf_traffic::DetectConflict::between(traits.profile(),
      traj1,
      traits.profile(),
      traj2).has_value();

  CHECK(detect_conflict);
}

//==============================================================================
SCENARIO("Rotation failure")
{
  SerializedWaypoint serialized_waypoint[4];
  serialized_waypoint[0] =
    read_waypoint(RESOURCES_DIR + std::string("/Waypoint1.txt"));
  serialized_waypoint[1] =
    read_waypoint(RESOURCES_DIR + std::string("/Waypoint2.txt"));
  serialized_waypoint[2] =
    read_waypoint(RESOURCES_DIR + std::string("/Waypoint3.txt"));
  serialized_waypoint[3] =
    read_waypoint(RESOURCES_DIR + std::string("/Waypoint4.txt"));

  rmf_traffic::agv::VehicleTraits traits{
    {0.5, 2.0}, {0.75, 1.5},
    rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex(
        rmf_traffic::geometry::Circle(0.2))
    }
  };

  rmf_traffic::Trajectory traj1, traj2;
  traj1.insert(
    serialized_waypoint[0].time,
    serialized_waypoint[0].position,
    serialized_waypoint[0].velocity);
  traj1.insert(
    serialized_waypoint[1].time,
    serialized_waypoint[1].position,
    serialized_waypoint[1].velocity);

  traj2.insert(
    serialized_waypoint[2].time,
    serialized_waypoint[2].position,
    serialized_waypoint[2].velocity);
  traj2.insert(
    serialized_waypoint[3].time,
    serialized_waypoint[3].position,
    serialized_waypoint[3].velocity);

  bool detect_conflict_with_rotation =
    rmf_traffic::DetectConflict::between(traits.profile(),
      traj1,
      traits.profile(),
      traj2).has_value();

  rmf_traffic::Trajectory traj3, traj4;
  traj3.insert(
    serialized_waypoint[0].time,
    {serialized_waypoint[0].position.x(),
      serialized_waypoint[0].position.y(),
      0.0},
    {serialized_waypoint[0].velocity.x(),
      serialized_waypoint[0].velocity.y(),
      0.0});
  traj3.insert(
    serialized_waypoint[1].time,
    {serialized_waypoint[1].position.x(),
      serialized_waypoint[1].position.y(),
      0.0},
    {serialized_waypoint[1].velocity.x(),
      serialized_waypoint[1].velocity.y(),
      0.0});

  traj4.insert(
    serialized_waypoint[2].time,
    {serialized_waypoint[2].position.x(),
      serialized_waypoint[2].position.y(),
      0.0},
    {serialized_waypoint[2].velocity.x(),
      serialized_waypoint[2].velocity.y(),
      0.0});
  traj4.insert(
    serialized_waypoint[3].time,
    {serialized_waypoint[3].position.x(),
      serialized_waypoint[3].position.y(),
      0.0},
    {serialized_waypoint[3].velocity.x(),
      serialized_waypoint[3].velocity.y(),
      0.0});


  bool detect_conflict_without_rotation =
    rmf_traffic::DetectConflict::between(traits.profile(),
      traj3,
      traits.profile(),
      traj4).has_value();

  CHECK(detect_conflict_with_rotation == detect_conflict_without_rotation);
}
}