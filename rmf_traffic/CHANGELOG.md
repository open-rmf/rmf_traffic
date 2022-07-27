## Changelog for package rmf_traffic

2.1.0 (XXXX-YY-ZZ)
------------------
* Quickest path feature: [#84](https://github.com/open-rmf/rmf_traffic/pull/84)

2.0.0 (2022-03-18)
------------------
* Introduce traffic dependency system: [#70](https://github.com/open-rmf/rmf_traffic/pull/70)

1.5.0 (2022-02-14)
------------------
* Support lane speed limits: [#44](https://github.com/open-rmf/rmf_traffic/pull/44)
* Fix potential race conditions: [#46](https://github.com/open-rmf/rmf_traffic/pull/46)
* Add features to facilitate robust failover: [#54](https://github.com/open-rmf/rmf_traffic/pull/54)
* Significantly improved performance for very large scale nav graphs: [#53](https://github.com/open-rmf/rmf_traffic/pull/53)

1.4.1 (2021-10-27)
------------------
* Using eigen3_cmake_module to fix RHEL build: [#47](https://github.com/open-rmf/rmf_traffic/pull/47)

1.4.0 (2021-09-01)
------------------
* Mandate use of FCL>=0.6: [#39](https://github.com/open-rmf/rmf_traffic/pull/39)
* Make the stubborn negotiator's strategy more flexible: [#40](https://github.com/open-rmf/rmf_traffic/pull/40)
* Fix participant lifecycles: [#35](https://github.com/open-rmf/rmf_traffic/pull/35)

1.3.0 (2021-05-07)
------------------
* Allow a Database to be forked off of a Mirror: [#17](https://github.com/open-rmf/rmf_traffic/pull/17)
* Separate participant descriptions from schedule patches: [#14](https://github.com/open-rmf/rmf_traffic/pull/14)
* Allow navigation graph lanes to be opened or closed: [#11](https://github.com/open-rmf/rmf_traffic/pull/11)
* Add persistence to Traffic Schedule Participant IDs: [#242](https://github.com/osrf/rmf_core/pull/242)
* Allow a minimum plan finish time to be specified: [#307](https://github.com/osrf/rmf_core/pull/307)
* Check itinerary endpoints when negotiating: [#308](https://github.com/osrf/rmf_core/pull/308)

1.2.0 (2021-01-05)
------------------
* Improve planner performance scaling for large graphs: [#243](https://github.com/osrf/rmf_core/pull/243)
* Add the blockade system for traffic light management: [#226](https://github.com/osrf/rmf_core/pull/226)
* Access trajectory waypoints by element index: [#226](https://github.com/osrf/rmf_core/pull/226)
* Get trajectory index of each plan waypoint: [#226](https://github.com/osrf/rmf_core/pull/226)

1.1.0 (2020-09-24)
------------------
* Allow a Negotiation Table Viewer to see rejected and forfeited statuses, and to check for a submission: [#140](https://github.com/osrf/rmf_core/pull/140/)
* Improve heuristic to account for events: [#159](https://github.com/osrf/rmf_core/pull/159/)
* Fix an issue with moving robots between floors: [#163](https://github.com/osrf/rmf_core/pull/163/)
* Add a generic waiting event: [#158](https://github.com/osrf/rmf_core/pull/158)
* Fix bug that caused exit events to get skipped sometimes: [#166](https://github.com/osrf/rmf_core/pull/166)
* Bump to C++17 and migrate to `std::optional`: [#177](https://github.com/osrf/rmf_core/pull/177)
* Contributors: Aaron Chong, Geoffrey Biggs, Grey, Kevin_Skywalker, Yadu, ddengster

1.0.2 (2020-07-27)
------------------
* Improved definition of "traffic conflict" for vechiles that start too close: [#136](https://github.com/osrf/rmf_core/pull/136)

1.0.1 (2020-07-20)
------------------
* Allow users to specify a callback for interrupting a planner: [#130](https://github.com/osrf/rmf_core/pull/130/)
* Allow a Negotiation Table Viewer to know when its Table is defunct: [#130](https://github.com/osrf/rmf_core/pull/130/)

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
