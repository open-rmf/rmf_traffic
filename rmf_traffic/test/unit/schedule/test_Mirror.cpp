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

#include "utils_Database.hpp"
#include <rmf_traffic/schedule/Database.hpp>

//#include <rmf_traffic/geometry/Box.hpp>
#include <src/rmf_traffic/geometry/Box.hpp>
#include <src/rmf_traffic/schedule/debug_Database.hpp>

#include "src/rmf_traffic/schedule/debug_Viewer.hpp"

#include <rmf_utils/catch.hpp>
#include <rmf_traffic/schedule/Mirror.hpp>
#include <rmf_traffic/DetectConflict.hpp>

#include <unordered_map>

using namespace std::chrono_literals;

using IdMap = std::unordered_map<
  rmf_traffic::schedule::ParticipantId,
  std::unordered_set<rmf_traffic::RouteId>>;

SCENARIO("Test Mirror of a Database with two trajectories")
{

  // Creating database db and checking for empty initialziation
  rmf_traffic::schedule::Database db;
  const auto query_all = rmf_traffic::schedule::query_all();
  rmf_traffic::schedule::Patch changes =
    db.changes(query_all, rmf_utils::nullopt);
  CHECK(changes.size() == 0);
  rmf_traffic::schedule::Version dbv = 0;
  CHECK(changes.latest_version() == dbv);

  // Add two participants
  double profile_scale = 1;
  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(profile_scale, profile_scale);
  const rmf_traffic::Profile profile{shape};

  const auto p1 = db.register_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "test_participant_1",
      "test_Mirror",
      rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
      profile
    });
  CHECK(db.latest_version() == ++dbv);
  rmf_traffic::schedule::ItineraryVersion iv1 = 0;
  rmf_traffic::PlanId pv1 = 0;
  rmf_traffic::schedule::Writer::StorageId sv1 = 0;

  const auto p2 = db.register_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "test_participant_2",
      "test_Mirror",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    });
  CHECK(db.latest_version() == ++dbv);
  rmf_traffic::schedule::ItineraryVersion iv2 = 0;
  rmf_traffic::PlanId pv2 = 0;
  rmf_traffic::schedule::Writer::StorageId sv2 = 0;

  // Creating Trajectories to insert
  const rmf_traffic::Time time = std::chrono::steady_clock::now();

  rmf_traffic::Trajectory t1;
  t1.insert(time, Eigen::Vector3d{-5, 0, 0}, Eigen::Vector3d{0, 0, 0});
  t1.insert(time + 10s, Eigen::Vector3d{5, 0, 0}, Eigen::Vector3d{0, 0, 0});
  REQUIRE(t1.size() == 2);

  rmf_traffic::Trajectory t2;
  t2.insert(time, Eigen::Vector3d{-5, 10, 0}, Eigen::Vector3d{0, 0, 0});
  t2.insert(time+10s, Eigen::Vector3d{5, 10, 0}, Eigen::Vector3d{0, 0, 0});
  REQUIRE(t2.size() == 2);

  db.set(p1.id(), pv1++, create_test_input(t1), sv1++, iv1++);
  CHECK(db.latest_version() == ++dbv);

  db.set(p2.id(), pv2++, create_test_input(t2), sv2++, iv2++);
  CHECK(db.latest_version() == ++dbv);
  REQUIRE_FALSE(rmf_traffic::DetectConflict::between(
      profile, t1, nullptr, profile, t2, nullptr));

  rmf_traffic::schedule::Mirror mirror;
  // updating mirror
  changes = db.changes(query_all, rmf_utils::nullopt);
  CHECK(mirror.update(changes));
  CHECK(mirror.latest_version() == changes.latest_version());

  rmf_traffic::schedule::ParticipantDescriptionsMap descriptions;
  for (const auto id : db.participant_ids())
    descriptions.insert_or_assign(id, *db.get_participant(id));
  mirror.update_participants_info(descriptions);

  WHEN("A trajectory is added to the database")
  {
    rmf_traffic::Trajectory t3;
    t3.insert(time, Eigen::Vector3d{-5, -10, 0}, Eigen::Vector3d{0, 0, 0});
    t3.insert(time+10s, Eigen::Vector3d{5, 10, 0}, Eigen::Vector3d{0, 0, 0});

    db.extend(p1.id(), create_test_input(t3), iv1++);
    ++sv1;
    CHECK(db.latest_version() == ++dbv);
    CHECK_TRAJECTORY_COUNT(db, 2, 3);
    CHECK(mirror.latest_version() != db.latest_version());

    THEN("Updating the mirror should update its latest version")
    {
      changes = db.changes(query_all, mirror.latest_version());
      mirror.update(changes);
      CHECK(mirror.latest_version() == db.latest_version());
      CHECK_TRAJECTORY_COUNT(mirror, 2, 3);
    }
  }

  GIVEN("Create a trajectory that conflicts with the schedule")
  {
    rmf_traffic::Trajectory t3;
    t3.insert(time, Eigen::Vector3d{0, -5, 0}, Eigen::Vector3d{0, 0, 0});
    t3.insert(time+10s, Eigen::Vector3d{0, 5, 0}, Eigen::Vector3d{0, 0, 0});

    auto view = mirror.query(query_all);
    auto conflicting_trajectories =
      get_conflicting_trajectories(view, profile, t3);
    CHECK(conflicting_trajectories.size() == 1);

    WHEN(
      "Replacing conflicting trajectory in db and updating mirror should eliminate conflict")
    {
      rmf_traffic::Trajectory t4;
      t4.insert(time, Eigen::Vector3d{-5, 0, 0}, Eigen::Vector3d{0, 0, 0});
      t4.insert(time+10s, Eigen::Vector3d{-2, 0, 0}, Eigen::Vector3d{0, 0, 0});

      db.set(p1.id(), pv1++, create_test_input(t4), sv1++, iv1++);
      CHECK(db.latest_version() == ++dbv);

      view = db.query(query_all);
      conflicting_trajectories =
        get_conflicting_trajectories(view, profile, t3);
      CHECK(conflicting_trajectories.size() == 0);

      changes = db.changes(query_all, mirror.latest_version());
      mirror.update(changes);
      CHECK(mirror.latest_version() == db.latest_version());

      view = mirror.query(query_all);
      conflicting_trajectories =
        get_conflicting_trajectories(view, profile, t3);
      CHECK(conflicting_trajectories.size() == 0);
    }

    WHEN(
      "Clearing conflicting trajectory in db and updating mirror should eliminate conflict")
    {
      db.clear(p1.id(), iv1++);
      CHECK(db.latest_version() == ++dbv);
      changes = db.changes(query_all, mirror.latest_version());
      mirror.update(changes);
      CHECK(mirror.latest_version() == db.latest_version());

      view = mirror.query(query_all);
      conflicting_trajectories =
        get_conflicting_trajectories(view, profile, t3);
      CHECK(conflicting_trajectories.size() == 0);
    }

    WHEN(
      "Delaying conflicting trajectory in db and updating mirror should eliminate conflict")
    {
      db.delay(p1.id(), 20s, iv1++);
      CHECK(db.latest_version() == ++dbv);
      changes = db.changes(query_all, mirror.latest_version());
      CHECK(mirror.update(changes));
      CHECK(mirror.latest_version() == db.latest_version());

      view = mirror.query(query_all);
      conflicting_trajectories =
        get_conflicting_trajectories(view, profile, t3);
      CHECK(conflicting_trajectories.size() == 0);
    }

    WHEN(
      "Culling conflicting trajectory in db and updating mirror should eliminate conflict")
    {
      db.cull(time + 11s);
      changes = db.changes(query_all, mirror.latest_version());
      CHECK(mirror.update(changes));
      CHECK(mirror.latest_version() == db.latest_version());

      view = mirror.query(query_all);
      conflicting_trajectories =
        get_conflicting_trajectories(view, profile, t3);
      CHECK(conflicting_trajectories.size() == 0);
    }
  }
}

SCENARIO("Testing specialized mirrors")
{
  rmf_traffic::schedule::Database db;
  rmf_traffic::schedule::Version dbv = 0;

  const auto query_all = rmf_traffic::schedule::query_all();
  rmf_traffic::schedule::Patch changes =
    db.changes(query_all, rmf_utils::nullopt);
  REQUIRE(changes.size() == 0);

  // Creating participants
  const double profile_scale = 1.0;
  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(profile_scale, profile_scale);
  const rmf_traffic::Profile profile{shape};

  const auto p1 = db.register_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant_1",
      "test_Mirror",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    });
  CHECK(db.latest_version() == ++dbv);
  rmf_traffic::schedule::ItineraryVersion iv1 = 0;
  rmf_traffic::PlanId pv1 = 0;
  rmf_traffic::schedule::Writer::StorageId sv1 = 0;

  const auto p2 = db.register_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant_2",
      "test_Mirror",
      rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
      profile
    });
  CHECK(db.latest_version() == ++dbv);
  rmf_traffic::schedule::ItineraryVersion iv2 = 0;
  rmf_traffic::PlanId pv2 = 0;
  rmf_traffic::schedule::Writer::StorageId sv2 = 0;

  //Creating routes r1, r2, r3 in "test_map"
  const rmf_traffic::Time time = std::chrono::steady_clock::now();

  rmf_traffic::Trajectory t1;
  t1.insert(time, {-5, 0, 0}, {0, 0, 0});
  t1.insert(time + 10s, {5, 0, 0}, {0, 0, 0});
  REQUIRE(t1.size() == 2);
  const auto r1 =rmf_traffic::Route("test_map", t1);

  rmf_traffic::Trajectory t2;
  t2.insert(time, {-5, 10, 0}, {0, 0, 0});
  t2.insert(time+11s, {5, 10, 0}, {0, 0, 0});
  REQUIRE(t2.size() == 2);
  const auto r2 = rmf_traffic::Route("test_map", t2);

  rmf_traffic::Trajectory t3;
  t3.insert(time+11s, {0, -5, 0}, {0, 0, 0});
  t3.insert(time+20s, {0, 5, 0}, {0, 0, 0});
  REQUIRE(t3.size() == 2);
  const auto r3 = rmf_traffic::Route("test_map", t3);

  // creating routes r4 and r5 in "test_map_2"
  rmf_traffic::Trajectory t4;
  t4.insert(time, {-5, 0, 0}, {0, 0, 0});
  t4.insert(time + 10s, {5, 0, 0}, {0, 0, 0});
  REQUIRE(t4.size() == 2);
  const auto r4 = rmf_traffic::Route("test_map_2", t4);

  rmf_traffic::Trajectory t5;
  t5.insert(time, {-5, 10, 0}, {0, 0, 0});
  t5.insert(time+10s, {5, 10, 0}, {0, 0, 0});
  REQUIRE(t5.size() == 2);
  const auto r5 = rmf_traffic::Route("test_map_2", t5);

  db.set(p1.id(), pv1++, {r1, r2, r4}, sv1, iv1++);
  sv1 += 3;
  CHECK(db.latest_version() == ++dbv);
  db.set(p2.id(), pv2++, {r3, r5}, sv2, iv2++);
  sv2 += 2;
  CHECK(db.latest_version() == ++dbv);

  // Check that there are no conflicts between the routes on test_map
  CHECK_FALSE(rmf_traffic::DetectConflict::between(
      profile, t1, nullptr, profile, t2, nullptr));
  CHECK_FALSE(rmf_traffic::DetectConflict::between(
      profile, t1, nullptr, profile, t3, nullptr));
  CHECK_FALSE(rmf_traffic::DetectConflict::between(
      profile, t2, nullptr, profile, t3, nullptr));

  // Check that there is no conflict between the routes on test_map_2
  CHECK_FALSE(rmf_traffic::DetectConflict::between(
      profile, t4, nullptr, profile, t5, nullptr));

  GIVEN("Query patch with spacetime region overlapping with t1")
  {
    auto time = std::chrono::steady_clock::now();
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
    rmf_traffic::schedule::Query query = rmf_traffic::schedule::make_query({});
    REQUIRE(query.spacetime().regions() != nullptr);

    //creating space to add to region
    const auto box = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Box>(10.0, 1.0);
    rmf_traffic::geometry::Space space(box, tf);
    std::vector<rmf_traffic::geometry::Space> spaces;
    spaces.push_back(space);
    //creating a region with time bounds and spaces that overlap with trajectory
    rmf_traffic::Region region("test_map", time, time+10s, spaces);
    query.spacetime().regions()->push_back(region);

    rmf_traffic::schedule::Patch changes =
      db.changes(query, rmf_utils::nullopt);

    REQUIRE(changes.size() > 0);
    CHECK(changes.size() == 1);
    CHECK(changes.begin()->participant_id() == p1.id());
    REQUIRE(changes.begin()->additions().items().size() == 1);
    CHECK(changes.begin()->additions().items().begin()->storage_id == 0);
  }

  GIVEN("Query patch with spacetime region overlapping with t2")
  {
    auto time = std::chrono::steady_clock::now();
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
    auto query = rmf_traffic::schedule::make_query({});
    REQUIRE(query.spacetime().regions() != nullptr);

    //creating space to add to region
    const auto box = rmf_traffic::geometry::Box(10.0, 1.0);
    const auto final_box = rmf_traffic::geometry::make_final_convex(box);
    tf.translate(Eigen::Vector2d{0.0, 10.0});
    rmf_traffic::geometry::Space space(final_box, tf);
    std::vector<rmf_traffic::geometry::Space> spaces;
    spaces.push_back(space);
    //creating a region with time bounds and spaces that overlap with trajectory
    rmf_traffic::Region region("test_map", time, time+10s, spaces);
    query.spacetime().regions()->push_back(region);

    //querying for Patch using defined spacetime query
    rmf_traffic::schedule::Patch changes =
      db.changes(query, rmf_utils::nullopt);

    REQUIRE(changes.size() > 0);
    CHECK(changes.size() == 1);
    CHECK(changes.begin()->participant_id() == p1.id());
    REQUIRE(changes.begin()->additions().items().size() == 1);
    CHECK(changes.begin()->additions().items().begin()->storage_id == 1);
  }

//  // COMMENTED DUE TO NON-DETERMINISTIC BEHAVIOR OF FCL

//  GIVEN("Query patch with rotated spacetime region overlapping with t1")
//  {
//    auto time = std::chrono::steady_clock::now();
//    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

//    rmf_traffic::schedule::Query query= rmf_traffic::schedule::make_query(0,{});
//    REQUIRE(query.spacetime().regions() != nullptr);

//    //creating space to add to region
//    const auto box = rmf_traffic::geometry::Box(10.0, 1.0);
//    const auto final_box = rmf_traffic::geometry::make_final_convex(box);
//    tf.rotate(Eigen::Rotation2Dd(M_PI_2));
//    rmf_traffic::geometry::Space space(final_box,tf);
//    std::vector<rmf_traffic::geometry::Space> spaces;
//    spaces.push_back(space);
//    //creating a region with time bounds and spaces that overlap with trajectory
//    rmf_traffic::Region region("test_map",time, time+10s,spaces);
//    query.spacetime().regions()->push_back(region);

//    //querying for Patch using defined spacetime query
//    rmf_traffic::schedule::Database::Patch changes= db.changes(query);
//    REQUIRE(changes.size() >0);
//    CHECK(changes.size() == 1);
//    CHECK(changes.begin()->id() == 1);
//  }

  GIVEN("Query patch with spacetime region overlapping with t3")
  {
    auto time = std::chrono::steady_clock::now();
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

    auto query = rmf_traffic::schedule::make_query({});
    REQUIRE(query.spacetime().regions() != nullptr);

    //creating space to add to region
    const auto box = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Box>(1.0, 1.0);
    rmf_traffic::geometry::Space space(box, tf);
    std::vector<rmf_traffic::geometry::Space> spaces;
    spaces.push_back(space);

    // creating a region with time bounds and spaces that overlap with trajectory
    rmf_traffic::Region region("test_map", time+10s, time+20s, spaces);
    query.spacetime().regions()->push_back(region);

    // querying for Patch using defined spacetime query
    rmf_traffic::schedule::Patch changes =
      db.changes(query, rmf_utils::nullopt);
    REQUIRE(changes.size() > 0);
    CHECK(changes.size() == 1);
    CHECK(changes.begin()->participant_id() == p2.id());
    REQUIRE(changes.begin()->additions().items().size() == 1);
    CHECK(changes.begin()->additions().items().begin()->storage_id == 0);
  }

  GIVEN("Query patch with spacetime region overlapping with t1 and t3")
  {
    auto time = std::chrono::steady_clock::now();
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

    auto query = rmf_traffic::schedule::make_query({});
    REQUIRE(query.spacetime().regions() != nullptr);

    // creating space to add to region
    const auto box = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Box>(10, 10);
    rmf_traffic::geometry::Space space(box, tf);
    std::vector<rmf_traffic::geometry::Space> spaces;
    spaces.push_back(space);

    //creating a region with time bounds and spaces that overlap with trajectory
    rmf_traffic::Region region("test_map", time, time+20s, spaces);
    query.spacetime().regions()->push_back(region);

    //querying for Patch using defined spacetime query
    rmf_traffic::schedule::Patch changes =
      db.changes(query, rmf_utils::nullopt);

    // Only t1 and t3 are within range of the region. t2 runs along the line y=10,
    // which is outside the range of a 10x10 box that is centered at the origin,
    // because that box will only reach out to y=5.
    REQUIRE(changes.size() > 0);
    CHECK(changes.size() == 2);

    IdMap ids;
    for (const auto& c : changes)
    {
      for (const auto& i : c.additions().items())
        ids[c.participant_id()].insert(i.storage_id);
    }

    IdMap expected_ids;
    expected_ids[p1.id()] = {0};
    expected_ids[p2.id()] = {0};

    CHECK(ids == expected_ids);
  }

  GIVEN("Query patch with spacetime region overlapping with t1, t2 & t3")
  {
    auto time = std::chrono::steady_clock::now();
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

    auto query = rmf_traffic::schedule::make_query({});
    REQUIRE(query.spacetime().regions() != nullptr);

    // creating space to add to region
    const auto box = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Box>(10, 20);
    rmf_traffic::geometry::Space space(box, tf);
    std::vector<rmf_traffic::geometry::Space> spaces;
    spaces.push_back(space);

    //creating a region with time bounds and spaces that overlap with trajectory
    rmf_traffic::Region region("test_map", time, time+20s, spaces);
    query.spacetime().regions()->push_back(region);

    // querying for Patch using defined spacetime query
    rmf_traffic::schedule::Patch changes =
      db.changes(query, rmf_utils::nullopt);

    REQUIRE(changes.size() > 0);
    CHECK(changes.size() == 2);

    IdMap ids;
    for (const auto& c : changes)
    {
      for (const auto& i : c.additions().items())
        ids[c.participant_id()].insert(i.storage_id);
    }

    IdMap expected_ids;
    expected_ids[p1.id()] = {0, 1};
    expected_ids[p2.id()] = {0};
  }
}

//==============================================================================
class FailoverWriter :
  public rmf_traffic::schedule::Writer,
  public std::enable_shared_from_this<FailoverWriter>
{
public:

  using RectFactory =
    rmf_traffic::schedule::DatabaseRectificationRequesterFactory;

  FailoverWriter()
  : _database(std::make_shared<rmf_traffic::schedule::Database>()),
    _rectifiers(std::make_shared<RectFactory>(_database))
  {
    // Do nothing
  }

  bool drop_packets = false;

  void set(
    ParticipantId participant,
    PlanId plan,
    const Itinerary& itinerary,
    StorageId storage_base,
    ItineraryVersion version) final
  {
    if (drop_packets)
      return;

    _database->set(participant, plan, itinerary, storage_base, version);
    _mirror.update(_database->changes(_all, _mirror.latest_version()));
  }

  void extend(
    ParticipantId participant,
    const Itinerary& routes,
    ItineraryVersion version) final
  {
    if (drop_packets)
      return;

    _database->extend(participant, routes, version);
    _mirror.update(_database->changes(_all, _mirror.latest_version()));
  }

  void delay(
    ParticipantId participant,
    Duration delay,
    ItineraryVersion version) final
  {
    if (drop_packets)
      return;

    _database->delay(participant, delay, version);
    _mirror.update(_database->changes(_all, _mirror.latest_version()));
  }

  void reached(
    ParticipantId participant,
    PlanId plan,
    const std::vector<CheckpointId>& reached_checkpoints,
    ProgressVersion version) final
  {
    if (drop_packets)
      return;

    _database->reached(participant, plan, reached_checkpoints, version);
    _mirror.update(_database->changes(_all, _mirror.latest_version()));
  }

  void clear(ParticipantId participant, ItineraryVersion version) final
  {
    if (drop_packets)
      return;

    _database->clear(participant, version);
    _mirror.update(_database->changes(_all, _mirror.latest_version()));
  }

  Registration register_participant(ParticipantDescription description) final
  {
    // We assume participant registration is done over a reliable connection
    auto registration = _database->register_participant(std::move(description));
    _update_participants();

    return registration;
  }

  void unregister_participant(ParticipantId participant) final
  {
    _database->set_current_time(std::chrono::steady_clock::now());
    _database->unregister_participant(participant);
    _update_participants();
  }

  void update_description(
    ParticipantId participant,
    ParticipantDescription desc) final
  {
    _database->update_description(participant, std::move(desc));
    _update_participants();
  }

  void failover()
  {
    _database =
      std::make_shared<rmf_traffic::schedule::Database>(_mirror.fork());
    _rectifiers->change_database(_database);
  }

  void rectify()
  {
    _rectifiers->rectify();
  }

  rmf_traffic::schedule::Participant make_participant(
    rmf_traffic::schedule::ParticipantDescription description)
  {
    return rmf_traffic::schedule::make_participant(
      std::move(description),
      shared_from_this(),
      _rectifiers);
  }

  const rmf_traffic::schedule::Database& database() const
  {
    return *_database;
  }

  const rmf_traffic::schedule::Mirror& mirror() const
  {
    return _mirror;
  }

private:

  void _update_participants()
  {
    rmf_traffic::schedule::ParticipantDescriptionsMap descriptions;
    for (const auto& p : _database->participant_ids())
      descriptions.insert_or_assign(p, *_database->get_participant(p));

    _mirror.update_participants_info(descriptions);
  }

  std::shared_ptr<rmf_traffic::schedule::Database> _database;
  rmf_traffic::schedule::Mirror _mirror;
  std::shared_ptr<rmf_traffic::schedule::DatabaseRectificationRequesterFactory>
  _rectifiers;

  rmf_traffic::schedule::Query _all = rmf_traffic::schedule::query_all();

};

//==============================================================================
SCENARIO("Test forking off of mirrors")
{
  // In this test, we create a writer which has the ability to "fail over",
  // meaning its back end database can be deleted and replaced with a new
  // database that forked off of a mirror that was keeping in sync with the
  // database. This writer also has packet loss issues that can be toggled on
  // and off.
  //
  // Given this writer, we will perform arbitrary operations on a participant
  // and intermittently toggle packet loss on and off, as well as trigger the
  // failover mechanism occasionally. Each time we do a failover, we will test
  // to make sure that the new back end database can continue staying in sync
  // with the participant, and that the new database agrees with the participant
  // about its current state.

  using namespace std::chrono_literals;
  const auto writer = std::make_shared<FailoverWriter>();

  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);

  const auto description = rmf_traffic::schedule::ParticipantDescription{
    "participant",
    "test_Mirror",
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    rmf_traffic::Profile{shape}
  };

  auto p0 = writer->make_participant(description);

  const auto now = std::chrono::steady_clock::now();

  rmf_traffic::Trajectory trajectory;
  trajectory.insert(now, {0, 0, 0}, {0, 0, 0});
  trajectory.insert(now + 10s, {0, 10, 0}, {0, 0, 0});

  const std::string test_map = "test_map";

  p0.set(p0.plan_id_assigner()->assign(), {{test_map, trajectory}});

  writer->drop_packets = true;

  trajectory.insert(now + 20s, {10, 10, 0}, {0, 0, 0});
  p0.extend({{test_map, trajectory}});

  writer->drop_packets = false;

  trajectory.insert(now + 22s, {10, 10, 0}, {0, 0, 0});
  p0.extend({{test_map, trajectory}});

  writer->rectify();

  p0.delay(10s);

  writer->drop_packets = true;

  trajectory.insert(now + 32s, {10, 0, 0}, {0, 0, 0});
  p0.extend({{test_map, trajectory}});
  p0.delay(5s);

  writer->failover();

  writer->drop_packets = false;

  trajectory.erase(trajectory.begin(), trajectory.end());
  trajectory.insert(now, {3, 2, 1}, {0, 0, 0});
  trajectory.insert(now + 10s, {1, 2, 3}, {0, 0, 0});
  p0.extend({{test_map, trajectory}});

  writer->rectify();

  {
    const auto database_itinerary = writer->database().get_itinerary(p0.id());
    REQUIRE(database_itinerary.has_value());
    REQUIRE(p0.itinerary().size() == database_itinerary->size());
    CHECK(p0.current_plan_id() ==
      writer->database().get_current_plan_id(p0.id()).value());
    CHECK(p0.current_plan_id() ==
      writer->mirror().get_current_plan_id(p0.id()).value());
  }

  p0.set(p0.plan_id_assigner()->assign(), {{test_map, trajectory}});

  writer->failover();

  // This tests to make sure the new Database's itinerary information
  // perfectly matches the participant's intended itinerary information.
  CHECK_ITINERARY(p0, writer->database());

  p0.clear();

  writer->drop_packets = true;

  trajectory.insert(now + 32s, {10, 0, 0}, {0, 0, 0});
  p0.set(p0.plan_id_assigner()->assign(), {{test_map, trajectory}});
  p0.delay(20s);

  trajectory.insert(now + 10s, {0, 10, 0}, {0, 0, 0});
  p0.extend({{test_map, trajectory}});

  writer->drop_packets = false;
  writer->rectify();

  CHECK_ITINERARY(p0, writer->database());
  CHECK(p0.current_plan_id() ==
    writer->database().get_current_plan_id(p0.id()).value());
  CHECK(p0.current_plan_id() ==
    writer->mirror().get_current_plan_id(p0.id()).value());

  writer->failover();

  CHECK_ITINERARY(p0, writer->database());

  writer->drop_packets = true;
  p0.clear();

  writer->failover();
  writer->drop_packets = false;
  writer->rectify();

  CHECK(rmf_traffic::schedule::Database::Debug::get_itinerary(
      writer->database(), p0.id())->empty());

  // TODO(MXG): This could use more testing. For example, what happens when
  // other mirrors were following the database and then it fails over while
  // the mirror is out of sync? How would their changesets be impacted?
}

