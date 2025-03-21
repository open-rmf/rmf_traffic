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

#ifndef RMF_TRAFFIC__AGV__GRAPH_HPP
#define RMF_TRAFFIC__AGV__GRAPH_HPP

#include <rmf_traffic/Time.hpp>

#include <Eigen/Geometry>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/clone_ptr.hpp>

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <iostream>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class Graph
{
public:

  /// Properties related to lifts (elevators) that exist in the graph
  class LiftProperties
  {
  public:
    /// Get the name of the lift.
    const std::string& name() const;

    /// Get the (x, y) location of the lift in RMF canonical coordinates.
    Eigen::Vector2d location() const;

    /// Get the orientation (in radians) of the lift in RMF canonical
    /// coordinates.
    double orientation() const;

    /// Get the dimensions of the lift, aligned with the lift's local (x, y)
    /// coordinates.
    Eigen::Vector2d dimensions() const;

    /// Get whether the specified position, given in RMF canonical coordinates,
    /// is inside the lift. The envelope will expand the footprint of the lift
    /// that is used in the calculation.
    bool is_in_lift(Eigen::Vector2d position, double envelope = 0.0) const;

    /// Constructor
    LiftProperties(
      std::string name,
      Eigen::Vector2d location,
      double orientations,
      Eigen::Vector2d dimensions);

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };
  using LiftPropertiesPtr = std::shared_ptr<LiftProperties>;

  class DoorProperties
  {
  public:
    /// Get the name of the door.
    const std::string& name() const;

    /// Get the start position of the door.
    Eigen::Vector2d start() const;

    /// Get the end position of the door.
    Eigen::Vector2d end() const;

    /// Get the name of the map that this door is on.
    const std::string& map() const;

    /// Check if the line formed by p0 -> p1 intersects this door.
    bool intersects(
      Eigen::Vector2d p0,
      Eigen::Vector2d p1,
      double envelope = 0.0) const;

    /// Constructor
    DoorProperties(
      std::string name,
      Eigen::Vector2d start,
      Eigen::Vector2d end,
      std::string map);

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };
  using DoorPropertiesPtr = std::shared_ptr<DoorProperties>;

  /// Properties assigned to each waypoint (vertex) in the graph
  class Waypoint
  {
  public:
    /// Get the name of the map that this Waypoint exists on.
    const std::string& get_map_name() const;

    /// Set the name of the map that this Waypoint exists on.
    Waypoint& set_map_name(std::string map);

    /// Get the position of this Waypoint
    const Eigen::Vector2d& get_location() const;

    /// Set the position of this Waypoint
    Waypoint& set_location(Eigen::Vector2d location);

    /// Returns true if this Waypoint can be used as a holding point for the
    /// vehicle, otherwise returns false.
    bool is_holding_point() const;

    /// Set whether this waypoint can be used as a holding point for the
    /// vehicle.
    Waypoint& set_holding_point(bool _is_holding_point);

    /// Returns true if this Waypoint is a passthrough point, meaning a planner
    /// should not have a robot wait at this point, even just briefly to allow
    /// another robot to pass. Setting passthrough points reduces the branching
    /// factor of a planner, allowing it to run faster, at the cost of losing
    /// possible solutions to conflicts.
    bool is_passthrough_point() const;

    /// Set this Waypoint to be a passthrough point.
    Waypoint& set_passthrough_point(bool _is_passthrough);

    /// Returns true if this Waypoint is a parking spot. Parking spots are used
    /// when an emergency alarm goes off, and the robot is required to park
    /// itself.
    bool is_parking_spot() const;

    /// Set this Waypoint to be a parking spot.
    Waypoint& set_parking_spot(bool _is_parking_spot);

    /// Returns true if this Waypoint is a charger spot. Robots are routed to
    /// these spots when their batteries charge levels drop below the threshold
    /// value.
    bool is_charger() const;

    /// Set this Waypoint to be a parking spot.
    Waypoint& set_charger(bool _is_charger);

    /// If this waypoint is inside the lift then this will return a pointer to
    /// the properties of the lift. Otherwise this will be a nullptr.
    LiftPropertiesPtr in_lift() const;

    /// Set the properties of the lift that the waypoint is inside of, or
    /// provide a nullptr if it is not inside a lift.
    Waypoint& set_in_lift(LiftPropertiesPtr properties);

    /// The index of this waypoint within the Graph. This cannot be changed
    /// after the waypoint is created.
    std::size_t index() const;

    /// If this waypoint has a name, return a reference to it. If this waypoint
    /// does not have a name, return a nullptr.
    ///
    /// The name of a waypoint can only be set using add_key() or set_key().
    const std::string* name() const;

    /// If this waypoint has a name, the name will be returned. Otherwise it
    /// will return the waypoint index, formatted into a string based on
    /// the index_format argument.
    ///
    /// \param[in] name_format
    ///   If this waypoint has an assigned name, the first instance of "%s"
    ///   within name_format will be replaced with the name of the waypoint. If
    ///   there is no %s in the name_format string, then this function will
    ///   simply return the name_format string as-is when the waypoint has a
    ///   name.
    ///
    /// \param[in] index_format
    ///   If this waypoint does not have an assigned name, the first instance of
    ///   "%d" within the index_format string will be replaced with the
    ///   stringified decimal index value of the waypoint. If there is no "%d"
    ///   in the index_format string, then this function will simply return the
    ///   index_format string as-is when the waypoint does not have a name.
    std::string name_or_index(
      const std::string& name_format = "%s",
      const std::string& index_format = "#%d") const;

    /// Get the mutex group that this waypoint is associated with. An empty
    /// string implies that it is not associated with any mutex group.
    ///
    /// Only one robot at a time is allowed to occupy any waypoint or lane
    /// associated with a particular mutex group.
    const std::string& in_mutex_group() const;

    /// Set what mutex group this waypoint is associated with. Passing in an
    /// empty string will disasscoiate the waypoint from any mutex group.
    Waypoint& set_in_mutex_group(std::string group_name);

    /// Get a merge radius specific to this waypoint, if it has one. The radius
    /// indicates that any robot within this distance of the waypoint can merge
    /// onto this waypoint.
    std::optional<double> merge_radius() const;

    /// Set the merge radius specific to this waypoint.
    Waypoint& set_merge_radius(std::optional<double> value);

    class Implementation;
  private:
    Waypoint();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// A class that implicitly specifies a constraint on the robot's orientation.
  class OrientationConstraint
  {
  public:

    /// Make an orientation constraint that requires a specific value for the
    /// orientation.
    static rmf_utils::clone_ptr<OrientationConstraint>
    make(std::vector<double> acceptable_orientations);

    enum class Direction
    {
      Forward,
      Backward,
    };

    /// Make an orientation constraint that requires the vehicle to face forward
    /// or backward.
    static rmf_utils::clone_ptr<OrientationConstraint>
    make(Direction direction, const Eigen::Vector2d& forward_vector);

    /// Apply the constraint to the given homogeneous position.
    ///
    /// \param[in,out] position
    ///   The position which needs to be constrained. The function should modify
    ///   this position such that it satisfies the constraint, if possible.
    ///
    /// \param[in] course_vector
    ///   The direction that the robot is travelling in. Given for informational
    ///   purposes.
    ///
    /// \return True if the constraint is satisfied with the new value of
    /// position. False if the constraint could not be satisfied.
    virtual bool apply(
      Eigen::Vector3d& position,
      const Eigen::Vector2d& course_vector) const = 0;

    /// Clone this OrientationConstraint.
    virtual rmf_utils::clone_ptr<OrientationConstraint> clone() const = 0;

    // Default destructor.
    virtual ~OrientationConstraint() = default;
  };

  /// Add a lane to connect two waypoints
  class Lane
  {
  public:

    /// A door in the graph which needs to be opened before a robot can enter a
    /// certain lane or closed before the robot can exit the lane.
    class Door
    {
    public:

      /// Constructor
      ///
      /// \param[in] name
      ///   Unique name of the door.
      ///
      /// \param[in] duration
      ///   How long the door takes to open or close.
      Door(std::string name, Duration duration);

      /// Get the unique name (ID) of this Door
      const std::string& name() const;

      /// Set the unique name (ID) of this Door
      Door& name(std::string name);

      /// Get the duration incurred by waiting for this door to open or close.
      Duration duration() const;

      /// Set the duration incurred by waiting for this door to open or close.
      Door& duration(Duration duration);

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    class DoorOpen : public Door { public: using Door::Door; };
    class DoorClose : public Door { public: using Door::Door; };

    /// A lift door in the graph which needs to be opened before a robot can
    /// enter a certain lane or closed before the robot can exit the lane.
    class LiftSession
    {
    public:

      /// Constructor
      ///
      /// \param[in] lift_name
      ///   Name of the lift that this door belongs to.
      ///
      /// \param[in] floor_name
      ///   Name of the floor that this door belongs to.
      ///
      /// \param[in] duration
      ///   How long the door takes to open or close.
      LiftSession(
        std::string lift_name,
        std::string floor_name,
        Duration duration);

      /// Get the name of the lift that the door belongs to
      const std::string& lift_name() const;

      /// Set the name of the lift that the door belongs to
      LiftSession& lift_name(std::string name);

      /// Get the name of the floor that this door is on
      const std::string& floor_name() const;

      /// Set the name of the floor that this door is on
      LiftSession& floor_name(std::string name);

      /// Get an estimate of how long it will take the door to open or close
      Duration duration() const;

      /// Set an estimate of how long it will take the door to open or close
      LiftSession& duration(Duration duration);

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    class LiftSessionBegin : public LiftSession
    {
    public:
      using LiftSession::LiftSession;
    };

    class LiftMove : public LiftSession
    {
    public:
      using LiftSession::LiftSession;
    };

    class LiftDoorOpen : public LiftSession
    {
    public:
      using LiftSession::LiftSession;
    };

    class LiftSessionEnd : public LiftSession
    {
    public:
      using LiftSession::LiftSession;
    };

    class Dock
    {
    public:

      /// Constructor
      ///
      /// \param[in]
      ///   Name of the dock that will be approached
      ///
      /// \param[in]
      ///   How long the robot will take to dock
      Dock(
        std::string dock_name,
        Duration duration);

      /// Get the name of the dock
      const std::string& dock_name() const;

      /// Set the name of the dock
      Dock& dock_name(std::string name);

      /// Get an estimate for how long the docking will take
      Duration duration() const;

      /// Set an estimate for how long the docking will take
      Dock& duration(Duration d);

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    class Wait
    {
    public:

      /// Constructor
      ///
      /// \param[in] duration
      ///   How long the wait will be.
      Wait(Duration value);

      /// Get how long the wait will be.
      Duration duration() const;

      /// Set how long the wait will be.
      Wait& duration(Duration value);

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// A customizable Executor that can carry out actions based on which Event
    /// type is present.
    class Executor
    {
    public:

      using DoorOpen = Lane::DoorOpen;
      using DoorClose = Lane::DoorClose;
      using LiftSessionBegin = Lane::LiftSessionBegin;
      using LiftDoorOpen = Lane::LiftDoorOpen;
      using LiftSessionEnd = Lane::LiftSessionEnd;
      using LiftMove = Lane::LiftMove;
      using Dock = Lane::Dock;
      using Wait = Lane::Wait;

      virtual void execute(const DoorOpen& open) = 0;
      virtual void execute(const DoorClose& close) = 0;
      virtual void execute(const LiftSessionBegin& begin) = 0;
      virtual void execute(const LiftDoorOpen& open) = 0;
      virtual void execute(const LiftSessionEnd& end) = 0;
      virtual void execute(const LiftMove& move) = 0;
      virtual void execute(const Dock& dock) = 0;
      virtual void execute(const Wait& wait) = 0;

      virtual ~Executor() = default;
    };

    class Event;
    using EventPtr = rmf_utils::clone_ptr<Event>;

    /// An abstraction for the different kinds of Lane events
    class Event
    {
    public:

      /// An estimate of how long the event will take
      virtual Duration duration() const = 0;

      template<typename DerivedExecutor>
      DerivedExecutor& execute(DerivedExecutor& executor) const
      {
        return static_cast<DerivedExecutor&>(
          execute(static_cast<Executor&>(executor)));
      }

      /// Execute this event
      virtual Executor& execute(Executor& executor) const = 0;

      /// Clone this event
      virtual EventPtr clone() const = 0;

      virtual ~Event() = default;

      static EventPtr make(DoorOpen open);
      static EventPtr make(DoorClose close);
      static EventPtr make(LiftSessionBegin open);
      static EventPtr make(LiftSessionEnd close);
      static EventPtr make(LiftMove move);
      static EventPtr make(LiftDoorOpen open);
      static EventPtr make(Dock dock);
      static EventPtr make(Wait wait);
    };


    /// A Lane Node wraps up a Waypoint with constraints. The constraints
    /// stipulate the conditions for entering or exiting the lane to reach this
    /// waypoint.
    class Node
    {
    public:

      /// Constructor
      ///
      /// \param waypoint_index
      ///   The index of the waypoint for this Node
      ///
      /// \param event
      ///   An event that must happen before/after this Node is approached
      ///   (before if it's an entry Node or after if it's an exit Node).
      ///
      /// \param orientation
      ///   Any orientation constraints for moving to/from this Node (depending
      ///   on whether it's an entry Node or an exit Node).
      Node(
        std::size_t waypoint_index,
        rmf_utils::clone_ptr<Event> event = nullptr,
        rmf_utils::clone_ptr<OrientationConstraint> orientation = nullptr);

      /// Constructor. The event parameter will be nullptr.
      ///
      /// \param waypoint_index
      ///   The index of the waypoint for this Node
      ///
      /// \param orientation
      ///   Any orientation constraints for moving to/from this Node (depending
      ///   on whether it's an entry Node or an exit Node).
      Node(
        std::size_t waypoint_index,
        rmf_utils::clone_ptr<OrientationConstraint> orientation);

      /// Get the index of the waypoint that this Node is wrapped around.
      std::size_t waypoint_index() const;

      /// Get a reference to an event that must occur before or after this Node
      /// is visited.
      ///
      /// \note Before if this is an entry node or after if this is an exit node
      const Event* event() const;

      /// Set the event that must occur before or after this Node is visited
      Node& event(rmf_utils::clone_ptr<Event> new_event);

      /// Get the constraint on orientation that is tied to this Node.
      const OrientationConstraint* orientation_constraint() const;

      class Implementation;
    private:
      // We make the Lane a friend so it can copy and move the Nodes
      friend class Lane;

      // These constructors are private to make sure a user can't modify the
      // waypoint_index of a Node by copying or moving
      Node(const Node&) = default;
      Node(Node&&) = default;
      Node& operator=(const Node&) = default;
      Node& operator=(Node&&) = default;
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };


    /// The Lane Properties class contains properties that apply across the full
    /// extent of the lane.
    class Properties
    {
    public:

      /// Construct a default set of properties
      /// * speed_limit: nullopt
      /// * mutex_group: ""
      Properties();

      /// Get the speed limit along this lane. If a std::nullopt is returned,
      /// then there is no specified speed limit for the lane.
      std::optional<double> speed_limit() const;

      /// Set the speed limit along this lane. Providing a std::nullopt
      /// indicates that there is no speed limit for the lane.
      Properties& speed_limit(std::optional<double> value);

      /// Get the mutex group that this lane is associated with. An empty string
      /// implies that it is not associated with any mutex group.
      ///
      /// Only one robot at a time is allowed to occupy any waypoint or lane
      /// associated with a particular mutex group.
      const std::string& in_mutex_group() const;

      /// Set what mutex group this lane is associated with. Passing in an
      /// empty string will disassociate the lane from any mutex group.
      Properties& set_in_mutex_group(std::string group_name);

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// Get the entry node of this Lane. The lane represents an edge in the
    /// graph that goes away from this node.
    Node& entry();

    /// const-qualified entry()
    const Node& entry() const;

    /// Get the exit node of this Lane. The lane represents an edge in the graph
    /// that goes into this node.
    Node& exit();

    /// const-qualified exit()
    const Node& exit() const;

    /// Get the properties of this Lane
    Properties& properties();

    /// const-qualified properties()
    const Properties& properties() const;

    /// Get the index of this Lane within the Graph.
    std::size_t index() const;

    class Implementation;
  private:
    Lane();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Default constructor
  Graph();

  /// Make a new waypoint for this graph. It will not be connected to any other
  /// waypoints until you use make_lane() to connect it.
  ///
  /// \note Waypoints cannot be erased from a Graph after they are created.
  Waypoint& add_waypoint(
    std::string map_name,
    Eigen::Vector2d location);

  /// Get a waypoint based on its index.
  Waypoint& get_waypoint(std::size_t index);

  /// const-qualified get_waypoint()
  const Waypoint& get_waypoint(std::size_t index) const;

  /// Find a waypoint given a key name. If the graph does not have a matching
  /// key name, then a nullptr will be returned.
  Waypoint* find_waypoint(const std::string& key);

  /// const-qualified find_waypoint()
  const Waypoint* find_waypoint(const std::string& key) const;

  /// Add a new waypoint key name to the graph. If a new key name is given, then
  /// this function will return true. If the given key name was already in use,
  /// then this will return false and nothing will be changed in the graph.
  bool add_key(const std::string& key, std::size_t wp_index);

  /// Remove the waypoint key with the given name, if it exists in this Graph.
  /// If the key was removed, this will return true. If the key did not exist,
  /// this will return false.
  bool remove_key(const std::string& key);

  /// Set a waypoint key. If this key is already in the Graph, it will be
  /// changed to the new association.
  ///
  /// This function will return false if wp_index is outside the range of the
  /// waypoints in this Graph.
  bool set_key(const std::string& key, std::size_t wp_index);

  /// Get the map of all keys in this Graph.
  const std::unordered_map<std::string, std::size_t>& keys() const;

  /// Get the number of waypoints in this Graph
  std::size_t num_waypoints() const;

  /// Make a lane for this graph. Lanes connect waypoints together, allowing the
  /// graph to know how the robot is allowed to traverse between waypoints.
  Lane& add_lane(
    const Lane::Node& entry,
    const Lane::Node& exit,
    Lane::Properties properties = Lane::Properties());

  /// Get the lane at the specified index
  Lane& get_lane(std::size_t index);

  /// const-qualified get_lane()
  const Lane& get_lane(std::size_t index) const;

  /// Get the number of Lanes in this Graph.
  std::size_t num_lanes() const;

  /// Get the indices of lanes that come out of the given Waypoint index
  const std::vector<std::size_t>& lanes_from(std::size_t wp_index) const;

  /// Get the indices of lanes that arrive into the given Waypoint index
  const std::vector<std::size_t>& lanes_into(std::size_t wp_index) const;

  /// Get a reference to the lane that goes from from_wp to to_wp if such a lane
  /// exists. If no such lane exists, this will return a nullptr. If multiple
  /// exist, this will return the one that was added most recently.
  Lane* lane_from(std::size_t from_wp, std::size_t to_wp);

  /// const-qualified lane_from()
  const Lane* lane_from(std::size_t from_wp, std::size_t to_wp) const;

  /// Add a known lift to the graph. If this lift has the same name as one
  /// previously added, we will continue to use the same pointer as the original
  /// and override the properties because lift names are expected to be unique.
  LiftPropertiesPtr set_known_lift(LiftProperties lift);

  /// Get all the known lifts.
  std::vector<LiftPropertiesPtr> all_known_lifts() const;

  /// Find a known lift based on its name.
  LiftPropertiesPtr find_known_lift(const std::string& name) const;

  /// Add a known door to the graph. If this door has the same name as one
  /// previously added, we will continue to use the same pointer as the original
  /// and override the properties because door names are expected to be unique.
  DoorPropertiesPtr set_known_door(DoorProperties door);

  /// Get all the known doors.
  std::vector<DoorPropertiesPtr> all_known_doors() const;

  /// Find a known door based on its name.
  DoorPropertiesPtr find_known_door(const std::string& name) const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__AGV__GRAPH_HPP
