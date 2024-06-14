^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_traffic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.3.3 (2024-06-15)
------------------

3.3.2 (2024-06-15)
------------------

3.3.1 (2023-12-22)
------------------
* Fix UB in plan squashing (`#106 <https://github.com/open-rmf/rmf_traffic/pull/106>`_)
* Contributors: Grey

3.3.0 (2023-12-15)
------------------
* New graph elements and various fixes (`#103 <https://github.com/open-rmf/rmf_traffic/pull/103>`_)

3.2.0 (2023-06-08)
------------------

3.1.0 (2023-06-06)
------------------
* Switch to rst changelogs (`#100 <https://github.com/open-rmf/rmf_traffic/pull/100>`_)
* Fix multi floor anomaly (`#97 <https://github.com/open-rmf/rmf_traffic/pull/97>`_)
* Fix end_versions initialization capacity error in ``NegotiatingRouteValidator::Generator::all()`` function (`#58 <https://github.com/open-rmf/rmf_traffic/pull/58>`_)
* Contributors: 0to1, Grey, Yadunund

3.0.0 (2022-10-03)
------------------
* Improve robustness of schedule failover: (`#88 <https://github.com/open-rmf/rmf_traffic/pull/88>`_)
* Allow participant profiles to be changed at runtime: (`#87 <https://github.com/open-rmf/rmf_traffic/pull/87>`_)
* Fix issues with schedule culling and incremental delays: (`#86 <https://github.com/open-rmf/rmf_traffic/pull/86>`_)
* Fix `dependency_resoution` typo: (`#82 <https://github.com/open-rmf/rmf_traffic/pull/82>`_)
* Quickest path feature: (`#84 <https://github.com/open-rmf/rmf_traffic/pull/84>`_) (`#85 <https://github.com/open-rmf/rmf_traffic/pull/85>`_)
* More graceful error handling: (`#71 <https://github.com/open-rmf/rmf_traffic/pull/71>`_) (`#76 <https://github.com/open-rmf/rmf_traffic/pull/76>`_) (`#80 <https://github.com/open-rmf/rmf_traffic/pull/80>`_) (`#81 <https://github.com/open-rmf/rmf_traffic/pull/81>`_)

2.0.0 (2022-03-18)
------------------
* Introduce traffic dependency system: (`#70 <https://github.com/open-rmf/rmf_traffic/pull/70>`_)

1.5.0 (2022-02-14)
------------------
* Support lane speed limits: (`#44 <https://github.com/open-rmf/rmf_traffic/pull/43>`_)
* Fix potential race conditions: (`#46 <https://github.com/open-rmf/rmf_traffic/pull/46>`_)
* Add features to facilitate robust failover: (`#54 <https://github.com/open-rmf/rmf_traffic/pull/54>`_)
* Significantly improved performance for very large scale nav graphs: (`#53 <https://github.com/open-rmf/rmf_traffic/pull/53>`_)

1.4.1 (2021-10-27)
------------------
* Using eigen3_cmake_module to fix RHEL build: (`#47 <https://github.com/open-rmf/rmf_traffic/pull/47>`_)

1.4.0 (2021-09-01)
------------------
* Mandate use of FCL>=0.6: (`#39 <https://github.com/open-rmf/rmf_traffic/pull/39>`_)
* Make the stubborn negotiator's strategy more flexible: (`#40 <https://github.com/open-rmf/rmf_traffic/pull/40>`_)
* Fix participant lifecycles: (`#35 <https://github.com/open-rmf/rmf_traffic/pull/35>`_)

1.3.0 (2021-05-07)
------------------
* Allow a Database to be forked off of a Mirror: (`#17 <https://github.com/open-rmf/rmf_traffic/pull/17>`_)
* Separate participant descriptions from schedule patches: (`#14 <https://github.com/open-rmf/rmf_traffic/pull/14>`_)
* Allow navigation graph lanes to be opened or closed: (`#11 <https://github.com/open-rmf/rmf_traffic/pull/11>`_)
* Add persistence to Traffic Schedule Participant IDs: (`#242 <https://github.com/osrf/rmf_core/pull/242>`_)
* Allow a minimum plan finish time to be specified: (`#307 <https://github.com/osrf/rmf_core/pull/307>`_)
* Check itinerary endpoints when negotiating: (`#308 <https://github.com/osrf/rmf_core/pull/308>`_)

1.2.0 (2021-01-05)
------------------
* Improve planner performance scaling for large graphs: (`#243 <https://github.com/osrf/rmf_core/pull/243>`_)
* Add the blockade system for traffic light management: (`#226 <https://github.com/osrf/rmf_core/pull/226>`_)

1.1.0 (2020-09-24)
------------------
* Allow a Negotiation Table Viewer to see rejected and forfeited statuses, and to check for a submission: (`#140 <https://github.com/osrf/rmf_core/pull/140>`_)
* Improve heuristic to account for events: (`#159 <https://github.com/osrf/rmf_core/pull/159>`_)
* Fix an issue with moving robots between floors: (`#163 <https://github.com/osrf/rmf_core/pull/163>`_)
* Add a generic waiting event: (`#158 <https://github.com/osrf/rmf_core/pull/158>`_)
* Fix bug that caused exit events to get skipped sometimes: (`#166 <https://github.com/osrf/rmf_core/pull/166>`_)
* Bump to C++17 and migrate to `std::optional`: (`#177 <https://github.com/osrf/rmf_core/pull/177>`_)
* Contributors: Aaron Chong, Geoffrey Biggs, Grey, Kevin_Skywalker, Yadu, ddengster

1.0.2 (2020-07-27)
------------------
* Improved definition of "traffic conflict" for vechiles that start too close: (`#136 <https://github.com/osrf/rmf_core/pull/136>`_)

1.0.1 (2020-07-20)
------------------
* Allow users to specify a callback for interrupting a planner: (`#130 <https://github.com/osrf/rmf_core/pull/130>`_)
* Allow a Negotiation Table Viewer to know when its Table is defunct: (`#130 <https://github.com/osrf/rmf_core/pull/130>`_)

1.0.0 (2020-06-23)
------------------
* Provides core `rmf_traffic` utilities
    * `Trajectory` - Describe a motion through 2D space
    * `Route` - Describe a path that a robot will follow
    * `Motion` - Convert a discrete `Trajectory` into a continuous function
* Provides `rmf_traffic::schedule` utilities for managing traffic schedules
    * `Database` - Object for managing a schedule database
    * `Viewer` - Interface for viewing a schedule database
    * `Writer` - Interface for writing to a schedule database
    * `Mirror` - Object for mirroring a schedule database across a distributed system
    * `Snapshot` - Object that captures a snapshot of a database
    * `Participant` - Object that manages participation in a schedule
    * `ParticipantDescription` - Object that describes a participant
    * `Query` - Object that describes a schedule query
    * `Negotiation` - Object that manages a traffic negotiation
    * `Negotiator` - Interface used to respond to negotiation events
    * `StubbornNegotiator` - An implementation of a `Negotiator` that refuses to deviate from its path
* Provides `rmf_traffic::agv` utilities to help AGV fleets integrate with the schedule
    * `Graph` - Describe the route graph that an AGV is allowed to use
    * `VehicleTraits` - Describe the kinematic properties of an AGV
    * `Interpolate` - Interpolate the trajectory of an AGV based on its traits
    * `RouteValidator` - Interface for determining whether a route is free of conflicts
    * `Planner` - Object that can generate plans for an AGV that comply with the schedule or that suit a negotiation
    * `SimpleNegotiator` - An implementation of a `schedule::Negotiator` that can negotiate for an AGV
* Contributors: Aaron Chong, Boon Han, Charayaphan Nakorn Boon Han, Grey, Luca Della Vedova, Marco A. Gutiérrez, Morgan Quigley, Yadu, Yadunund, koonpeng
