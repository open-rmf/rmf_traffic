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

#include "utils_Database.hpp"

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include "src/rmf_traffic/schedule/debug_Database.hpp"
#include "src/rmf_traffic/schedule/debug_Participant.hpp"

#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_utils/catch.hpp>

#include <unordered_map>

class FaultyWriter : public rmf_traffic::schedule::Writer
{
public:

  FaultyWriter(std::shared_ptr<rmf_traffic::schedule::Database> database)
  : _database(std::move(database))
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
  }

  void extend(
    rmf_traffic::schedule::ParticipantId participant,
    const Itinerary& routes,
    rmf_traffic::schedule::ItineraryVersion version) final
  {
    if (drop_packets)
      return;

    _database->extend(participant, routes, version);
  }

  void delay(
    rmf_traffic::schedule::ParticipantId participant,
    rmf_traffic::Duration delay,
    rmf_traffic::schedule::ItineraryVersion version) final
  {
    if (drop_packets)
      return;

    _database->delay(participant, delay, version);
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
  }

  void clear(
    rmf_traffic::schedule::ParticipantId participant,
    rmf_traffic::schedule::ItineraryVersion version) final
  {
    if (drop_packets)
      return;

    _database->clear(participant, version);
  }

  Registration register_participant(
    rmf_traffic::schedule::ParticipantDescription participant_info) final
  {
    // We assume participant registration is done over a reliable connection
    return _database->register_participant(participant_info);
  }

  void unregister_participant(
    rmf_traffic::schedule::ParticipantId participant) final
  {
    _database->set_current_time(std::chrono::steady_clock::now());
    _database->unregister_participant(participant);
  }

  void update_description(
    rmf_traffic::schedule::ParticipantId participant,
    rmf_traffic::schedule::ParticipantDescription desc) final
  {
    _database->update_description(participant, std::move(desc));
  }

private:
  std::shared_ptr<rmf_traffic::schedule::Database> _database;
};

//=============================================================================
// Symbol key for the test scenarios below
//
// S: accepted set change
// s: dropped set change
//
// X: accepted extend change
// x: dropped extend change
//
// D: accepted delay change
// d: dropped delay change
//
// E: accepted earse change
// e: dropped erase change
//
// C: accepted clear change
// c: dropped clear change
//=============================================================================

SCENARIO("Test Participant")
{
  using namespace std::chrono_literals;
  using Route = rmf_traffic::Route;

  // Create a database
  auto db = std::make_shared<rmf_traffic::schedule::Database>();
  rmf_traffic::schedule::Version dbv = 0;
  CHECK(db->latest_version() == dbv);
  REQUIRE(db->participant_ids().size() == 0);
  REQUIRE(db->get_participant(0) == nullptr);
  REQUIRE(db->inconsistencies().size() == 0);

  // Create a faulty writer
  const auto writer = std::make_shared<FaultyWriter>(db);

  // Create a simple rectifier factory
  const auto rectifier = std::make_shared<
    rmf_traffic::schedule::DatabaseRectificationRequesterFactory>(db);

  // Create a shape
  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);

  // Create a participant
  auto p1 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant 1",
      "test_Participant",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      rmf_traffic::Profile{shape}
    },
    writer,
    rectifier);

  CHECK(p1.id() == 0);
  const auto& description = p1.description();
  CHECK(description.name() == "participant 1");
  CHECK(description.owner() == "test_Participant");
  CHECK(description.responsiveness() == rmf_traffic::schedule
    ::ParticipantDescription::Rx::Responsive);
  REQUIRE(p1.itinerary().size() == 0);

  CHECK(db->participant_ids().size() == 1);
  REQUIRE(db->participant_ids().count(p1.id()) == 1);
  CHECK(db->latest_version() == ++dbv);
  REQUIRE(db->inconsistencies().begin() != db->inconsistencies().end());
  REQUIRE(db->get_itinerary(p1.id()));
  CHECK(db->get_itinerary(p1.id())->empty());

  const auto time = std::chrono::steady_clock::now();
  rmf_traffic::Trajectory t1;
  t1.insert(time, {0, 0, 0}, {0, 0, 0});
  t1.insert(time + 10s, {0, 10, 0}, {0, 0, 0});
  REQUIRE(t1.size() == 2);

  rmf_traffic::Trajectory t2;
  t2.insert(time + 12s, {0, 10, 0}, {0, 0, 0});
  t2.insert(time + 20s, {10, 10, 0}, {0, 0, 0});
  REQUIRE(t2.size() == 2);

  rmf_traffic::Trajectory t3;
  t3.insert(time + 22s, {10, 10, 0}, {0, 0, 0});
  t3.insert(time + 32s, {10, 0, 0}, {0, 0, 0});
  REQUIRE(t3.size() == 2);

  GIVEN("Changes: S")
  {
    const auto plan_id = p1.plan_id_assigner()->assign();
    p1.set(plan_id, {Route{"test_map", t1}});
    CHECK(p1.current_plan_id() == plan_id);
    CHECK(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == ++dbv);
    CHECK_ITINERARY(p1, *db);

    WHEN("When participant itinerary is set with an empty itinerary")
    {
      // This should be the same as getting cleared
      p1.set(p1.plan_id_assigner()->assign(), {});
      CHECK(p1.itinerary().empty());

      CHECK(db->latest_version() == ++dbv);
      CHECK_ITINERARY(p1, *db);
    }
  }

  GIVEN("Changes: SS")
  {
    p1.set(p1.plan_id_assigner()->assign(), {Route{"test_map", t1}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == ++dbv);
    CHECK_ITINERARY(p1, *db);

    p1.set(p1.plan_id_assigner()->assign(),
        {
          Route{"test_map_2", t1},
          Route{"test_map_3", t2}
        });

    CHECK(p1.itinerary().size() == 2);
    CHECK(db->latest_version() == ++dbv);
    CHECK_ITINERARY(p1, *db);
  }

  GIVEN("Changes: X")
  {
    p1.extend({Route{"test_map", t1}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == ++dbv);
    CHECK_ITINERARY(p1, *db);
  }

  GIVEN("Changes: SX")
  {
    p1.set(p1.plan_id_assigner()->assign(), {Route{"test_map", t1}});
    REQUIRE(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == ++dbv);
    CHECK_ITINERARY(p1, *db);

    // Extend itinerary with two new routes
    std::vector<Route> additional_routes;
    additional_routes.emplace_back(Route{"test_map", t2});
    additional_routes.emplace_back(Route{"test_map_2", t1});

    p1.extend(additional_routes);
    CHECK(p1.itinerary().size() == 3);
    CHECK(db->latest_version() == ++dbv);
    CHECK_ITINERARY(p1, *db);
  }

  GIVEN("Changes: D")
  {
    p1.delay(5s);
    CHECK(p1.itinerary().size() == 0);
    CHECK(p1.current_plan_id() ==
      std::numeric_limits<rmf_traffic::PlanId>::max());

    // We do not need to transmit a delay when the itinerary is empty
    CHECK(db->latest_version() == dbv);
  }

  GIVEN("Changes: SD")
  {
    p1.set(p1.plan_id_assigner()->assign(), {Route{"test_map", t1}});
    REQUIRE(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == ++dbv);
    CHECK_ITINERARY(p1, *db);

    // We make a copy of the itinerary for comparison after delay
    const auto old_itinerary = p1.itinerary();

    const auto delay_duration = 5s;
    p1.delay(delay_duration);
    REQUIRE(p1.itinerary().size() == 1);

    auto old_it = old_itinerary.front().trajectory().begin();
    auto new_it = p1.itinerary().front().trajectory().begin();

    for (; new_it != p1.itinerary().front().trajectory().end();
      new_it++, old_it++)
    {
      CHECK((new_it->time() - (old_it->time() + delay_duration)).count()
        == Approx(0.0));
    }

    CHECK(db->latest_version() == ++dbv);
    CHECK_ITINERARY(p1, *db);
  }

  GIVEN("Changes: C")
  {
    // Clearing an empty itinerary
    p1.clear();
    CHECK(p1.current_plan_id() ==
      std::numeric_limits<rmf_traffic::PlanId>::max());
    CHECK(p1.itinerary().size() == 0);
    // No change in database version
    CHECK(db->latest_version() == dbv);
  }

  GIVEN("Changes: SC")
  {
    p1.set(
      p1.plan_id_assigner()->assign(),
      {Route{"test_map", t1}, Route{"test_map_2", t2}});
    REQUIRE(p1.itinerary().size() == 2);
    CHECK(db->latest_version() == ++dbv);
    CHECK_ITINERARY(p1, *db);

    p1.clear();
    CHECK(p1.itinerary().size() == 0);
    CHECK(db->latest_version() == ++dbv);
    CHECK_ITINERARY(p1, *db);
  }

  GIVEN("Changes: s")
  {
    writer->drop_packets = true;

    p1.set(p1.plan_id_assigner()->assign(), {{"test_map", t1}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == dbv);
    REQUIRE(db->get_itinerary(p1.id()));
    CHECK(db->get_itinerary(p1.id())->empty());
  }

  GIVEN("Changes sS")
  {
    writer->drop_packets = true;

    p1.set(p1.plan_id_assigner()->assign(), {{"test_map", t1}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == dbv);
    REQUIRE(db->get_itinerary(p1.id()));
    CHECK(db->get_itinerary(p1.id())->empty());

    writer->drop_packets = false;

    p1.set(p1.plan_id_assigner()->assign(), {Route{"test_map", t2}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db->get_itinerary(p1.id())->size() == 1);
    CHECK(db->latest_version() == ++dbv);
    CHECK(db->inconsistencies().begin()->participant == p1.id());
    CHECK(db->inconsistencies().begin()->ranges.size() == 0);

    rectifier->rectify();
    // There is no need to fix anything, because the last change was a
    // nullifying change.
    CHECK(db->latest_version() == dbv);
    CHECK_ITINERARY(p1, *db);
    CHECK(db->inconsistencies().begin()->ranges.size() == 0);
  }

  GIVEN("Changes sDdS")
  {
    writer->drop_packets = true;

    p1.set(p1.plan_id_assigner()->assign(), {{"test_map", t1}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == dbv);
    REQUIRE(db->get_itinerary(p1.id()));
    CHECK(db->get_itinerary(p1.id())->empty());

    writer->drop_packets = false;

    p1.delay(10s);
    CHECK(db->latest_version() == dbv);
    REQUIRE(db->inconsistencies().size() == 1);
    CHECK(db->inconsistencies().begin()->ranges.size() == 1);
    CHECK(db->inconsistencies().begin()->ranges.last_known_version() ==
      rmf_traffic::schedule::Participant::Debug::get_itinerary_version(p1));

    writer->drop_packets = true;

    p1.delay(10s);
    CHECK(db->latest_version() == dbv);
    CHECK(db->inconsistencies().begin()->ranges.last_known_version() + 1 ==
      rmf_traffic::schedule::Participant::Debug::get_itinerary_version(p1));

    writer->drop_packets = false;

    p1.set(p1.plan_id_assigner()->assign(), {{"test_map", t2}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == ++dbv);
    REQUIRE(db->get_itinerary(p1.id()));
    CHECK(db->get_itinerary(p1.id())->size() == 1);
    CHECK(db->inconsistencies().begin()->ranges.size() == 0);
    CHECK(db->inconsistencies().begin()->ranges.last_known_version() ==
      rmf_traffic::schedule::Participant::Debug::get_itinerary_version(p1));

    rectifier->rectify();
    // There is no need to fix anything, because the last change was a
    // nullifying change.
    CHECK(db->latest_version() == dbv);
    CHECK_ITINERARY(p1, *db);
  }

  GIVEN("Changes: SRRRR(R)SR")
  {
    const auto plan_1 = p1.assign_plan_id();
    p1.set(plan_1, {{"test_map", t1}, {"test_map", t2}});
    CHECK(db->latest_version() == ++dbv);

    const auto* progress = db->get_current_progress(p1.id());
    REQUIRE(progress);
    CHECK(progress->size() == 2);
    for (const auto& c : *progress)
      CHECK(c == 0);

    p1.reached(plan_1, 0, 3);
    CHECK(db->latest_version() == ++dbv);

    progress = db->get_current_progress(p1.id());
    REQUIRE(progress);
    REQUIRE(progress->size() == 2);
    CHECK((*progress)[0] == 3);
    CHECK((*progress)[1] == 0);

    p1.reached(plan_1, 5, 4);
    CHECK(db->latest_version() == ++dbv);

    progress = db->get_current_progress(p1.id());
    REQUIRE(progress);
    REQUIRE(progress->size() == 6);
    CHECK((*progress)[0] == 3);
    CHECK((*progress)[1] == 0);
    CHECK((*progress)[2] == 0);
    CHECK((*progress)[3] == 0);
    CHECK((*progress)[4] == 0);
    CHECK((*progress)[5] == 4);

    bool reached_1_0_2 = false;
    bool deprecated_1_0_2 = false;
    const auto watch_1_0_2 = db->watch_dependency(
      rmf_traffic::Dependency{p1.id(), plan_1, 0, 2},
      [&]() { reached_1_0_2 = true; },
      [&]() { deprecated_1_0_2 = true; });

    // watch_dependency should immediately trigger the on_reached function.
    CHECK(reached_1_0_2);
    CHECK_FALSE(deprecated_1_0_2);

    bool reached_1_5_4 = false;
    bool deprecated_1_5_4 = false;
    const auto watch_1_5_4 = db->watch_dependency(
      rmf_traffic::Dependency{p1.id(), plan_1, 5, 4},
      [&]() { reached_1_5_4 = true; },
      [&]() { deprecated_1_5_4 = true; });

    // watch_dependency should immediately trigger the on_reached function.
    CHECK(reached_1_5_4);
    CHECK_FALSE(deprecated_1_5_4);
    CHECK(watch_1_5_4.reached());
    CHECK_FALSE(watch_1_5_4.deprecated());

    bool reached_1_3_6 = false;
    bool deprecated_1_3_6 = false;
    const auto watch_1_3_6 = db->watch_dependency(
      rmf_traffic::Dependency{p1.id(), plan_1, 3, 6},
      [&]() { reached_1_3_6 = true; },
      [&]() { deprecated_1_3_6 = true; });
    CHECK_FALSE(reached_1_3_6);
    CHECK_FALSE(deprecated_1_3_6);
    CHECK_FALSE(watch_1_3_6.reached());
    CHECK_FALSE(watch_1_3_6.deprecated());

    bool reached_1_3_7 = false;
    bool deprecated_1_3_7 = false;
    const auto watch_1_3_7 = db->watch_dependency(
      rmf_traffic::Dependency{p1.id(), plan_1, 3, 7},
      [&]() { reached_1_3_7 = true; },
      [&]() { deprecated_1_3_7 = true; });
    CHECK_FALSE(reached_1_3_7);
    CHECK_FALSE(deprecated_1_3_7);
    CHECK_FALSE(watch_1_3_7.reached());
    CHECK_FALSE(watch_1_3_7.deprecated());

    bool reached_1_3_8 = false;
    bool deprecated_1_3_8 = false;
    const auto watch_1_3_8 = db->watch_dependency(
      rmf_traffic::Dependency{p1.id(), plan_1, 3, 8},
      [&]() { reached_1_3_8 = true; },
      [&]() { deprecated_1_3_8 = true; });
    CHECK_FALSE(reached_1_3_8);
    CHECK_FALSE(deprecated_1_3_8);
    CHECK_FALSE(watch_1_3_8.reached());
    CHECK_FALSE(watch_1_3_8.deprecated());

    // Double-check this right before the change
    CHECK_FALSE(reached_1_3_6);
    CHECK_FALSE(deprecated_1_3_6);

    // Now trigger an arrival
    p1.reached(plan_1, 3, 6);
    CHECK(reached_1_3_6);
    CHECK_FALSE(deprecated_1_3_6);
    CHECK(watch_1_3_6.reached());
    CHECK_FALSE(watch_1_3_6.deprecated());
    CHECK_FALSE(reached_1_3_7);
    CHECK_FALSE(deprecated_1_3_7);
    CHECK_FALSE(watch_1_3_7.reached());
    CHECK_FALSE(watch_1_3_7.deprecated());
    CHECK_FALSE(reached_1_3_8);
    CHECK_FALSE(deprecated_1_3_8);
    CHECK_FALSE(watch_1_3_8.reached());
    CHECK_FALSE(watch_1_3_8.deprecated());

    p1.reached(plan_1, 3, 8);
    CHECK(reached_1_3_7);
    CHECK_FALSE(deprecated_1_3_7);
    CHECK(watch_1_3_7.reached());
    CHECK_FALSE(watch_1_3_7.deprecated());
    CHECK(reached_1_3_8);
    CHECK_FALSE(deprecated_1_3_8);
    CHECK(watch_1_3_8.reached());
    CHECK_FALSE(watch_1_3_8.deprecated());

    const auto plan_2 = p1.assign_plan_id() + 100;
    const auto skipped_plan = plan_1 + 5;

    bool reached_s_0_2 = false;
    bool deprecated_s_0_2 = false;
    const auto watch_s_0_2 = db->watch_dependency(
      rmf_traffic::Dependency{p1.id(), skipped_plan, 0, 2},
      [&]() { reached_s_0_2 = true; },
      [&]() { deprecated_s_0_2 = true; });
    CHECK_FALSE(reached_s_0_2);
    CHECK_FALSE(deprecated_s_0_2);

    bool reached_2_1_3 = false;
    bool deprecated_2_1_3 = false;
    const auto watch_2_1_3 = db->watch_dependency(
      rmf_traffic::Dependency{p1.id(), plan_2, 1, 3},
      [&]() { reached_2_1_3 = true; },
      [&]() { deprecated_2_1_3 = true; });
    CHECK_FALSE(reached_2_1_3);
    CHECK_FALSE(deprecated_2_1_3);

    bool reached_2_2_1 = false;
    bool deprecated_2_2_1 = false;
    const auto watch_2_2_1 = db->watch_dependency(
      rmf_traffic::Dependency{p1.id(), plan_2, 2, 1},
      [&]() { reached_2_2_1 = true; },
      [&]() { deprecated_2_2_1 = true; });
    CHECK_FALSE(reached_2_2_1);
    CHECK_FALSE(deprecated_2_2_1);

    p1.reached(plan_2, 1, 3);
    CHECK_FALSE(reached_2_1_3);
    CHECK_FALSE(deprecated_2_1_3);

    p1.set(plan_2, {{"test_map", t1}, {"test_map", t2}, {"test_map", t3}});
    CHECK_FALSE(reached_s_0_2);
    CHECK(deprecated_s_0_2);
    CHECK(reached_2_1_3);
    CHECK_FALSE(deprecated_2_1_3);
    CHECK_FALSE(reached_2_2_1);
    CHECK_FALSE(deprecated_2_2_1);

    p1.reached(plan_2, 2, 1);
    CHECK(reached_2_2_1);
    CHECK_FALSE(deprecated_2_2_1);

    bool reached_s_1_3 = false;
    bool deprecated_s_1_3 = false;
    const auto watch_s_1_3 = db->watch_dependency(
      rmf_traffic::Dependency{p1.id(), skipped_plan+1, 1, 3},
      [&]() { reached_s_1_3 = true; },
      [&]() { deprecated_s_1_3 = true; });
    CHECK_FALSE(reached_s_1_3);
    CHECK(deprecated_s_1_3);
  }

  GIVEN("Changes: srSRsr")
  {
    writer->drop_packets = true;

    const auto plan_1 = p1.assign_plan_id();
    const auto watch_1_0_2 = db->watch_dependency(
      rmf_traffic::Dependency{p1.id(), plan_1, 0, 2}, []() {}, []() {});

    p1.set(plan_1, {{"test_map", t1}});
    p1.reached(plan_1, 0, 2);

    CHECK_FALSE(watch_1_0_2.reached());
    CHECK_FALSE(watch_1_0_2.deprecated());

    writer->drop_packets = false;
    rectifier->rectify();

    CHECK(watch_1_0_2.reached());
    CHECK_FALSE(watch_1_0_2.deprecated());

    const auto plan_2 = p1.assign_plan_id();
    bool reached_2_1_1 = false;
    bool deprecated_2_1_1 = false;
    using Subscription =
      rmf_traffic::schedule::ItineraryViewer::DependencySubscription;
    std::optional<Subscription> watch_2_1_1 = db->watch_dependency(
      rmf_traffic::Dependency{p1.id(), plan_2, 1, 1},
      [&]() { reached_2_1_1 = true; },
      [&]() { deprecated_2_1_1 = true; });

    p1.set(plan_2, {{"test_map", t2}});

    // Drop the watch
    watch_2_1_1 = std::nullopt;

    p1.reached(plan_2, 1, 1);

    // Since the watch was dropped, these should still both be false
    CHECK_FALSE(reached_2_1_1);
    CHECK_FALSE(deprecated_2_1_1);

    const auto plan_3 = p1.assign_plan_id();
    const auto watch_3_2_1 = db->watch_dependency(
      rmf_traffic::Dependency{p1.id(), plan_3, 2, 1}, []() {}, []() {});

    const auto plan_4 = p1.assign_plan_id();
    const auto watch_4_3_2 = db->watch_dependency(
      rmf_traffic::Dependency{p1.id(), plan_4, 3, 2}, []() {}, []() {});

    writer->drop_packets = true;
    p1.set(plan_3, {{"test_map", t2}});
    p1.reached(plan_3, 2, 1);

    CHECK_FALSE(watch_3_2_1.reached());
    CHECK_FALSE(watch_3_2_1.deprecated());

    p1.set(plan_4, {{"test_map", t3}});
    p1.reached(plan_4, 3, 2);

    CHECK_FALSE(watch_3_2_1.reached());
    CHECK_FALSE(watch_3_2_1.deprecated());
    CHECK_FALSE(watch_4_3_2.reached());
    CHECK_FALSE(watch_4_3_2.deprecated());

    writer->drop_packets = false;
    rectifier->rectify();

    CHECK_FALSE(watch_3_2_1.reached());
    CHECK(watch_3_2_1.deprecated());
    CHECK(watch_4_3_2.reached());
    CHECK_FALSE(watch_4_3_2.deprecated());
  }

  GIVEN("Changes: sX")
  {
    writer->drop_packets = true;

    // Set the itinerary
    p1.set(
      p1.plan_id_assigner()->assign(),
      {Route{"test_map", t1}, Route{"test_map", t2}});
    CHECK(db->latest_version() == dbv);

    writer->drop_packets = false;

    // Extend the itinerary
    p1.extend({Route{"test_map_2", t2}});
    REQUIRE(db->get_itinerary(p1.id()));
    CHECK(db->get_itinerary(p1.id())->size() == 0);
    CHECK(db->inconsistencies().begin()->participant == p1.id());
    CHECK(db->inconsistencies().begin()->ranges.size() != 0);

    // Fix inconsistencies
    rectifier->rectify();
    CHECK(db->latest_version() == ++(++dbv));
    CHECK_ITINERARY(p1, *db);
    CHECK(db->inconsistencies().begin()->ranges.size() == 0);
  }

  GIVEN("Changes: SxX")
  {
    p1.set(
      p1.plan_id_assigner()->assign(),
      {Route{"test_map", t1}});

    CHECK(db->latest_version() == ++dbv);
    CHECK_ITINERARY(p1, *db);

    // Tell the writer to start dropping packets
    writer->drop_packets = true;

    p1.extend({Route{"test_map_2", t1}});

    // Check that the database version did not change
    CHECK(db->latest_version() == dbv);

    writer->drop_packets = false;

    p1.extend({Route{"test_map_3", t1}});

    // Check that the database version still did not change
    CHECK(db->latest_version() == dbv);

    // Check that the database now sees that we have an inconsistency
    REQUIRE(db->inconsistencies().size() > 0);
    CHECK(db->inconsistencies().begin()->participant == p1.id());
    CHECK(db->inconsistencies().begin()->ranges.size() != 0);

    // Tell the rectifier to fix the inconsistencies
    rectifier->rectify();

    // Now the database should have updated with both changes
    CHECK(db->latest_version() == ++(++dbv));
    REQUIRE(db->get_itinerary(p1.id()));
    CHECK(db->get_itinerary(p1.id())->size() == 3);
    const auto itinerary = db->get_itinerary(p1.id());
    CHECK_ITINERARY(p1, *db);
    CHECK(db->inconsistencies().begin()->ranges.size() == 0);
  }

  GIVEN("Changes: sddxX")
  {
    writer->drop_packets = true;

    // Set the participant itinerary
    p1.set(p1.plan_id_assigner()->assign(), {Route{"test_map", t1}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == dbv);
    REQUIRE(db->get_itinerary(p1.id()));
    CHECK(db->get_itinerary(p1.id())->empty());

    // Add a delay to the itinerary
    p1.delay(1s);
    CHECK(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == dbv);
    REQUIRE(db->get_itinerary(p1.id()));
    CHECK(db->get_itinerary(p1.id())->empty());

    // Add a second delay to the itinerary
    p1.delay(1s);
    CHECK(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == dbv);
    REQUIRE(db->get_itinerary(p1.id()));
    CHECK(db->get_itinerary(p1.id())->empty());

    // Extend the itinerary
    p1.extend({Route{"test_map", t2}, Route{"test_map", t3}});
    CHECK(p1.itinerary().size() == 3);
    CHECK(db->latest_version() == dbv);
    REQUIRE(db->get_itinerary(p1.id()));
    CHECK(db->get_itinerary(p1.id())->empty());

    writer->drop_packets = false;

    // Extend the itinerary
    p1.extend({Route{"test_map_2", t3}});
    CHECK(p1.itinerary().size() == 4);
    CHECK(db->latest_version() == dbv);
    REQUIRE(db->get_itinerary(p1.id()));
    CHECK(db->get_itinerary(p1.id())->empty());

    // Check for inconsistencies
    REQUIRE(db->inconsistencies().size() > 0);
    CHECK(db->inconsistencies().begin()->participant == p1.id());
    const auto inconsistency = db->inconsistencies().begin();
    CHECK(inconsistency->ranges.size() == 1);
    CHECK(inconsistency->ranges.last_known_version() == 4);
    CHECK(inconsistency->ranges.begin()->lower == 0);
    CHECK(inconsistency->ranges.begin()->upper == 3);

    // Fix inconsistencies
    rectifier->rectify();
    dbv += 5;
    CHECK(db->latest_version() == dbv);
    CHECK_ITINERARY(p1, *db);
    CHECK(db->inconsistencies().begin()->ranges.size() == 0);
  }

  GIVEN("Changes: SdDxX")
  {
    // Set the participant itinerary
    p1.set(p1.plan_id_assigner()->assign(), {Route{"test_map", t1}});
    REQUIRE(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == ++dbv);
    CHECK_ITINERARY(p1, *db);

    writer->drop_packets = true;

    // Add a delay
    p1.delay(1s);
    CHECK(db->latest_version() == dbv);

    writer->drop_packets = false;

    // Add a delay
    p1.delay(1s);
    CHECK(db->latest_version() == dbv);
    REQUIRE(db->inconsistencies().size() > 0);
    auto inconsistency = db->inconsistencies().begin();
    CHECK(inconsistency->participant == p1.id());
    REQUIRE(inconsistency->ranges.size() > 0);
    CHECK(inconsistency->ranges.last_known_version() == 2);
    CHECK(inconsistency->ranges.begin()->lower == 1);
    CHECK(inconsistency->ranges.begin()->upper == 1);

    writer->drop_packets = true;

    // Extend the itinerary
    p1.extend({Route{"test_map", t2}});
    CHECK(p1.itinerary().size() == 2);
    CHECK(db->latest_version() == dbv);

    writer->drop_packets = false;

    // Extend the itinerary
    p1.extend({Route{"test_map", t3}});
    REQUIRE(db->inconsistencies().size() > 0);
    CHECK(db->inconsistencies().begin()->participant == p1.id());
    inconsistency = db->inconsistencies().begin();
    // We expect two ranges of inconsistencies
    REQUIRE(inconsistency->ranges.size() > 1);
    CHECK(inconsistency->ranges.last_known_version() == 4);
    auto it = inconsistency->ranges.begin();
    CHECK(it->lower == 1);
    CHECK(it->upper == 1);
    ++it;
    REQUIRE(it != inconsistency->ranges.end());
    CHECK(it->lower == 3);
    CHECK(it->upper == 3);

    // Fix inconsistencies
    rectifier->rectify();
    dbv += 4;
    CHECK(db->latest_version() == dbv);
    CHECK_ITINERARY(p1, *db);
    CHECK(db->inconsistencies().begin()->ranges.size() == 0);
  }

  GIVEN("Changes: ScX")
  {
    // Set the participant itinerary
    p1.set(p1.plan_id_assigner()->assign(), {Route{"test_map", t1}});
    REQUIRE(p1.itinerary().size() == 1);
    CHECK(db->latest_version() == ++dbv);
    CHECK_ITINERARY(p1, *db);

    writer->drop_packets = true;

    // Clear the itinerary
    p1.clear();
    REQUIRE(p1.itinerary().size() == 0);
    CHECK(db->latest_version() == dbv);
    REQUIRE(db->get_itinerary(p1.id()));
    CHECK(db->get_itinerary(p1.id())->size() == 1);

    writer->drop_packets = false;

    // Extend the itinerary
    p1.extend({Route{"test_map", t2}, Route{"test_map", t3}});
    REQUIRE(p1.itinerary().size() == 2);
    CHECK(db->latest_version() == dbv);

    // Check for inconsistencies
    REQUIRE(db->inconsistencies().size() == 1);
    auto inconsistency = db->inconsistencies().begin();
    CHECK(inconsistency->participant == p1.id());
    REQUIRE(inconsistency->ranges.size() > 0);
    CHECK(inconsistency->ranges.last_known_version() == 2);
    CHECK(inconsistency->ranges.begin()->lower == 1);
    CHECK(inconsistency->ranges.begin()->upper == 1);

    // Fix inconsistency
    rectifier->rectify();
    dbv += 2;
    CHECK(db->latest_version() == dbv);
    CHECK_ITINERARY(p1, *db);
    CHECK(db->inconsistencies().begin()->ranges.size() == 0);
  }

  GIVEN("Participant unregisters")
  {

    rmf_traffic::schedule::ParticipantId p2_id;

    {
      auto p2 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant 2",
          "test_Participant",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          rmf_traffic::Profile{shape}
        },
        writer,
        rectifier);

      p2_id = p2.id();
      CHECK(db->latest_version() == ++dbv);
      CHECK(db->participant_ids().size() == 2);
      CHECK(db->get_participant(p2_id));
      REQUIRE(db->get_itinerary(p2_id));
      REQUIRE(db->inconsistencies().size() == 2);

      bool found = false;
      for (const auto& inconsistency : db->inconsistencies())
      {
        if (inconsistency.participant == p2_id)
        {
          found = true;
          CHECK(inconsistency.ranges.size() == 0);
        }
      }
      REQUIRE(found);
    }

    // Participant p2 should be unregistered when it is out of scope
    CHECK(db->latest_version() == ++dbv);
    CHECK(db->participant_ids().size() == 1);
    CHECK(db->get_participant(p2_id) == nullptr);
    CHECK_FALSE(db->get_itinerary(p2_id).has_value());
    CHECK(db->inconsistencies().size() == 1);
  }
}
