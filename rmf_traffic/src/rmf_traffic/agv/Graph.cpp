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

#include "internal_Graph.hpp"

#include <rmf_traffic/agv/Graph.hpp>

#include <rmf_utils/math.hpp>
#include <rmf_utils/optional.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class Graph::LiftProperties::Implementation
{
public:
  std::string name;
  Eigen::Vector2d location;
  double orientation;
  Eigen::Vector2d half_dimensions;
  Eigen::Isometry2d tf_inv;

  static void update(
    LiftProperties& original,
    const LiftProperties& incoming)
  {
    *original._pimpl = *incoming._pimpl;
  }
};

//==============================================================================
const std::string& Graph::LiftProperties::name() const
{
  return _pimpl->name;
}

//==============================================================================
Eigen::Vector2d Graph::LiftProperties::location() const
{
  return _pimpl->location;
}

//==============================================================================
double Graph::LiftProperties::orientation() const
{
  return _pimpl->orientation;
}

//==============================================================================
Eigen::Vector2d Graph::LiftProperties::dimensions() const
{
  return 2.0 * _pimpl->half_dimensions;
}

//==============================================================================
bool Graph::LiftProperties::is_in_lift(
  Eigen::Vector2d position,
  double envelope) const
{
  Eigen::Vector2d p_local = _pimpl->tf_inv * position;
  for (int i = 0; i < 2; ++i)
  {
    if (p_local[i]  < -_pimpl->half_dimensions[i] - envelope)
      return false;

    if (_pimpl->half_dimensions[i] + envelope < p_local[i])
      return false;
  }

  return true;
}

//==============================================================================
Eigen::Isometry2d make_lift_tf_inv(Eigen::Vector2d location, double orientation)
{
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
  tf.translate(location);
  tf.rotate(orientation);
  return tf.inverse();
}

//==============================================================================
Graph::LiftProperties::LiftProperties(
  std::string name,
  Eigen::Vector2d location,
  double orientation,
  Eigen::Vector2d dimensions)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation {
        std::move(name),
        location,
        orientation,
        dimensions / 2.0,
        make_lift_tf_inv(location, orientation)
      }))
{
  // Do nothing
}

//==============================================================================
class Graph::DoorProperties::Implementation
{
public:
  std::string name;
  Eigen::Vector2d start;
  Eigen::Vector2d end;
  std::string map;
};

//==============================================================================
const std::string& Graph::DoorProperties::name() const
{
  return _pimpl->name;
}

//==============================================================================
Eigen::Vector2d Graph::DoorProperties::start() const
{
  return _pimpl->start;
}

//==============================================================================
Eigen::Vector2d Graph::DoorProperties::end() const
{
  return _pimpl->end;
}

//==============================================================================
const std::string& Graph::DoorProperties::map() const
{
  return _pimpl->map;
}

//==============================================================================
namespace {
double distance_from_point_to_segment(
  Eigen::Vector2d q,
  Eigen::Vector2d p0,
  Eigen::Vector2d p1)
{
  const double endpoint_distance = std::min((q - p0).norm(), (q - p1).norm());
  const auto L = (p1 - p0).norm();
  if (L < 1e-3)
  {
    return endpoint_distance;
  }

  const Eigen::Vector2d n = (p1 - p0) / L;
  const Eigen::Vector2d v = q - p0;
  const Eigen::Vector2d q_proj = (v.dot(n)) * n;
  const double ortho_distance = (q - q_proj).norm();
  return std::min(ortho_distance, endpoint_distance);
}
} // anonymous namespace

//==============================================================================
bool Graph::DoorProperties::intersects(
  Eigen::Vector2d p0,
  Eigen::Vector2d p1,
  double envelope) const
{
  const auto q0 = _pimpl->start;
  const auto q1 = _pimpl->end;
  for (const auto test : std::vector<std::function<double()>>{
      [&]{ return distance_from_point_to_segment(p0, q0, q1); },
      [&]{ return distance_from_point_to_segment(p1, q0, q1); },
      [&]{ return distance_from_point_to_segment(q0, p0, p1); },
      [&]{ return distance_from_point_to_segment(_pimpl->end, p0, p1); }
    })
  {
    const double distance = test();
    if (distance <= envelope)
      return true;
  }

  // If none of the endpoints are within range of the other lines, then the only
  // way for an intersection to exist is if the lines truly cross each other.
  const double det = (p0.x() - p1.x()) * (q0.y() - q1.y())
    - (p0.y() - p1.y()) * (q0.x() - q1.x());

  if (std::abs(det) < 1e-8)
  {
    // The lines are essentially parallel and their endpoints aren't close
    // enough, so there is no intersection.
    return false;
  }

  const double t = ( (p0.x() - q0.x()) * (q0.y() - q1.y())
    - (p0.y() - q0.y()) * (q0.x() - q1.x()) )
    / det;

  if (t < 0.0 || 1.0 < t)
    return false;

  const double u = ( (p0.x() - q0.x()) * (p0.y() - p1.y())
    - (p0.y() - q0.y()) * (p0.x() - p1.x()) )
    / det;

  if (u < 0.0 || 1.0 < u)
    return false;

  return true;
}

//==============================================================================
Graph::DoorProperties::DoorProperties(
  std::string name,
  Eigen::Vector2d start,
  Eigen::Vector2d end,
  std::string map)
: _pimpl(rmf_utils::make_impl<Implementation>(
    Implementation {
      std::move(name),
      start,
      end,
      std::move(map)
    }))
{
  // Do nothing
}

//==============================================================================
class Graph::Waypoint::Implementation
{
public:

  std::size_t index;

  std::string map_name;

  Eigen::Vector2d location;

  rmf_utils::optional<std::string> name = rmf_utils::nullopt;

  bool holding_point = false;

  bool passthrough_point = false;

  bool parking_spot = false;

  bool charger = false;

  LiftPropertiesPtr in_lift = nullptr;

  std::string mutex_group = "";

  std::optional<double> merge_radius = std::nullopt;

  template<typename... Args>
  static Waypoint make(Args&& ... args)
  {
    Waypoint result;
    result._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{std::forward<Args>(args)...});

    return result;
  }

  static Waypoint::Implementation& get(Waypoint& wp)
  {
    return *wp._pimpl;
  }
};

//==============================================================================
const std::string& Graph::Waypoint::get_map_name() const
{
  return _pimpl->map_name;
}

//==============================================================================
auto Graph::Waypoint::set_map_name(std::string map) -> Waypoint&
{
  _pimpl->map_name = std::move(map);
  return *this;
}

//==============================================================================
const Eigen::Vector2d& Graph::Waypoint::get_location() const
{
  return _pimpl->location;
}

//==============================================================================
auto Graph::Waypoint::set_location(Eigen::Vector2d location) -> Waypoint&
{
  _pimpl->location = std::move(location);
  return *this;
}

//==============================================================================
bool Graph::Waypoint::is_holding_point() const
{
  return _pimpl->holding_point;
}

//==============================================================================
auto Graph::Waypoint::set_holding_point(bool _is_holding_point) -> Waypoint&
{
  _pimpl->holding_point = _is_holding_point;
  return *this;
}

//==============================================================================
bool Graph::Waypoint::is_passthrough_point() const
{
  return _pimpl->passthrough_point;
}

//==============================================================================
auto Graph::Waypoint::set_passthrough_point(bool _is_passthrough) -> Waypoint&
{
  _pimpl->passthrough_point = _is_passthrough;
  return *this;
}

//==============================================================================
bool Graph::Waypoint::is_parking_spot() const
{
  return _pimpl->parking_spot;
}

//==============================================================================
auto Graph::Waypoint::set_parking_spot(bool _is_parking_spot) -> Waypoint&
{
  _pimpl->parking_spot = _is_parking_spot;
  return *this;
}

//==============================================================================
bool Graph::Waypoint::is_charger() const
{
  return _pimpl->charger;
}

//==============================================================================
auto Graph::Waypoint::set_charger(bool _is_charger) -> Waypoint&
{
  _pimpl->charger = _is_charger;
  return *this;
}

//==============================================================================
auto Graph::Waypoint::in_lift() const -> LiftPropertiesPtr
{
  return _pimpl->in_lift;
}

//==============================================================================
auto Graph::Waypoint::set_in_lift(LiftPropertiesPtr lift) -> Waypoint&
{
  _pimpl->in_lift = lift;
  return *this;
}

//==============================================================================
std::size_t Graph::Waypoint::index() const
{
  return _pimpl->index;
}

//==============================================================================
const std::string* Graph::Waypoint::name() const
{
  if (_pimpl->name)
    return &_pimpl->name.value();

  return nullptr;
}

//==============================================================================
std::string Graph::Waypoint::name_or_index(
  const std::string& name_format,
  const std::string& index_format) const
{
  if (_pimpl->name)
  {
    const auto it = name_format.find_first_of("%s");
    if (it == std::string::npos)
      return name_format;

    return name_format.substr(0, it)
      + _pimpl->name.value()
      + name_format.substr(it+2);
  }

  const auto it = index_format.find_first_of("%d");
  if (it == std::string::npos)
    return index_format;

  return index_format.substr(0, it)
    + std::to_string(_pimpl->index)
    + index_format.substr(it+2);
}

//==============================================================================
const std::string& Graph::Waypoint::in_mutex_group() const
{
  return _pimpl->mutex_group;
}

//==============================================================================
auto Graph::Waypoint::set_in_mutex_group(std::string group_name) -> Waypoint&
{
  _pimpl->mutex_group = std::move(group_name);
  return *this;
}

//==============================================================================
std::optional<double> Graph::Waypoint::merge_radius() const
{
  return _pimpl->merge_radius;
}

//==============================================================================
auto Graph::Waypoint::set_merge_radius(std::optional<double> value) -> Waypoint&
{
  _pimpl->merge_radius = value;
  return *this;
}

//==============================================================================
Graph::Waypoint::Waypoint()
{
  // Do nothing
}

namespace {
//==============================================================================
class AcceptableOrientationConstraint : public Graph::OrientationConstraint
{
public:

  AcceptableOrientationConstraint(std::vector<double> acceptable)
  : orientations(std::move(acceptable))
  {
    // Do nothing
  }

  std::vector<double> orientations;

  bool apply(Eigen::Vector3d& position,
    const Eigen::Vector2d& /*course_vector*/) const final
  {
    assert(!orientations.empty());
    // This constraint can never be satisfied if there are no acceptable
    // orientations.
    if (orientations.empty())
      return false;

    const double p = position[2];
    double closest = p;
    double best_diff = std::numeric_limits<double>::infinity();
    for (const double theta : orientations)
    {
      const double diff = std::abs(rmf_utils::wrap_to_pi(theta - p));
      if (diff < best_diff)
      {
        closest = theta;
        best_diff = diff;
      }
    }

    position[2] = closest;
    return true;
  }

  rmf_utils::clone_ptr<OrientationConstraint> clone() const final
  {
    return rmf_utils::make_clone<AcceptableOrientationConstraint>(*this);
  }

};

//==============================================================================
// TODO(MXG): Think about how to refactor this constraint so that it can share
// an implementation with DifferentialOrientationConstraint. Maybe instead of
// a single direction it could have a std::vector of acceptable directions.
// Or it can have `bool forward_okay` and `bool backward_okay` fields.
class DirectionConstraint : public Graph::OrientationConstraint
{
public:

  static Eigen::Rotation2Dd compute_forward_offset(
    const Eigen::Vector2d& forward)
  {
    return Eigen::Rotation2Dd(std::atan2(forward[1], forward[0]));
  }

  static const Eigen::Rotation2Dd R_pi;

  DirectionConstraint(
    Direction _direction,
    const Eigen::Vector2d& _forward_vector)
  : R_f(compute_forward_offset(_forward_vector)),
    R_f_inv(R_f.inverse()),
    direction(_direction)
  {
    // Do nothing
  }

  Eigen::Rotation2Dd R_f;
  Eigen::Rotation2Dd R_f_inv;
  Direction direction;

  Eigen::Rotation2Dd compute_R_final(
    const Eigen::Vector2d& course_vector) const
  {
    const Eigen::Rotation2Dd R_c(
      std::atan2(course_vector[1], course_vector[0]));

    if (Direction::Backward == direction)
      return R_pi * R_c * R_f_inv;

    return R_c * R_f_inv;
  }

  bool apply(
    Eigen::Vector3d& position,
    const Eigen::Vector2d& course_vector) const final
  {
    position[2] = rmf_utils::wrap_to_pi(compute_R_final(course_vector).angle());
    return true;
  }

  rmf_utils::clone_ptr<OrientationConstraint> clone() const final
  {
    return rmf_utils::make_clone<DirectionConstraint>(*this);
  }
};

//==============================================================================
const Eigen::Rotation2Dd DirectionConstraint::R_pi = Eigen::Rotation2Dd(M_PI);

} // anonymous namespace

//==============================================================================
rmf_utils::clone_ptr<Graph::OrientationConstraint>
Graph::OrientationConstraint::make(std::vector<double> acceptable_orientations)
{
  return rmf_utils::make_clone<AcceptableOrientationConstraint>(
    std::move(acceptable_orientations));
}

//==============================================================================
rmf_utils::clone_ptr<Graph::OrientationConstraint>
Graph::OrientationConstraint::make(
  Direction direction,
  const Eigen::Vector2d& forward)
{
  return rmf_utils::make_clone<DirectionConstraint>(direction, forward);
}

//==============================================================================
class Graph::Lane::Door::Implementation
{
public:

  std::string name;
  Duration duration;

};

//==============================================================================
Graph::Lane::Door::Door(
  std::string name,
  Duration duration)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(name),
        duration
      }))
{
  // Do nothing
}

//==============================================================================
const std::string& Graph::Lane::Door::name() const
{
  return _pimpl->name;
}

//==============================================================================
auto Graph::Lane::Door::name(std::string name) -> Door&
{
  _pimpl->name = std::move(name);
  return *this;
}

//==============================================================================
Duration Graph::Lane::Door::duration() const
{
  return _pimpl->duration;
}

//==============================================================================
auto Graph::Lane::Door::duration(Duration duration_) -> Door&
{
  _pimpl->duration = duration_;
  return *this;
}

//==============================================================================
class Graph::Lane::LiftSession::Implementation
{
public:

  std::string lift_name;
  std::string floor_name;
  Duration duration;

};

//==============================================================================
Graph::Lane::LiftSession::LiftSession(
  std::string lift_name,
  std::string floor_name,
  Duration duration)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(lift_name),
        std::move(floor_name),
        duration
      }))
{
  // Do nothing
}

//==============================================================================
const std::string& Graph::Lane::LiftSession::lift_name() const
{
  return _pimpl->lift_name;
}

//==============================================================================
auto Graph::Lane::LiftSession::lift_name(std::string name) -> LiftSession&
{
  _pimpl->lift_name = std::move(name);
  return *this;
}

//==============================================================================
const std::string& Graph::Lane::LiftSession::floor_name() const
{
  return _pimpl->floor_name;
}

//==============================================================================
auto Graph::Lane::LiftSession::floor_name(std::string name) -> LiftSession&
{
  _pimpl->floor_name = std::move(name);
  return *this;
}

//==============================================================================
Duration Graph::Lane::LiftSession::duration() const
{
  return _pimpl->duration;
}

//==============================================================================
auto Graph::Lane::LiftSession::duration(Duration duration_) -> LiftSession&
{
  _pimpl->duration = duration_;
  return *this;
}

//==============================================================================
class Graph::Lane::Dock::Implementation
{
public:

  std::string dock_name;
  Duration duration;

};

//==============================================================================
Graph::Lane::Dock::Dock(
  std::string dock_name,
  Duration duration)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(dock_name),
        duration
      }))
{
  // Do nothing
}

//==============================================================================
const std::string& Graph::Lane::Dock::dock_name() const
{
  return _pimpl->dock_name;
}

//==============================================================================
auto Graph::Lane::Dock::dock_name(std::string name) -> Dock&
{
  _pimpl->dock_name = name;
  return *this;
}

//==============================================================================
Duration Graph::Lane::Dock::duration() const
{
  return _pimpl->duration;
}

//==============================================================================
auto Graph::Lane::Dock::duration(Duration d) -> Dock&
{
  _pimpl->duration = d;
  return *this;
}

//==============================================================================
class Graph::Lane::Wait::Implementation
{
public:

  Duration duration;

};

//==============================================================================
Graph::Lane::Wait::Wait(Duration value)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{value}))
{
  // Do nothing
}

//==============================================================================
Duration Graph::Lane::Wait::duration() const
{
  return _pimpl->duration;
}

//==============================================================================
auto Graph::Lane::Wait::duration(Duration value) -> Wait&
{
  _pimpl->duration = value;
  return *this;
}

//==============================================================================
void Graph::Lane::Executor::execute(const Wait&)
{
  // Do nothing
}

namespace {
//==============================================================================
template<typename EventT>
class TemplateEvent : public Graph::Lane::Event
{
public:

  using This = TemplateEvent<EventT>;

  TemplateEvent(EventT event)
  : _event(std::move(event))
  {
    // Do nothing
  }

  Duration duration() const final
  {
    return _event.duration();
  }

  Graph::Lane::Executor& execute(Graph::Lane::Executor& executor) const final
  {
    executor.execute(_event);
    return executor;
  }

  static Graph::Lane::EventPtr make(EventT _event)
  {
    return rmf_utils::make_clone<This>(std::move(_event));
  }

  Graph::Lane::EventPtr clone() const final
  {
    return make(_event);
  }

private:

  EventT _event;

};
} // anonymous namespace

//==============================================================================
auto Graph::Lane::Event::make(DoorOpen open) -> EventPtr
{
  return TemplateEvent<DoorOpen>::make(std::move(open));
}

//==============================================================================
auto Graph::Lane::Event::make(DoorClose close) -> EventPtr
{
  return TemplateEvent<DoorClose>::make(std::move(close));
}

//==============================================================================
auto Graph::Lane::Event::make(LiftSessionBegin open) -> EventPtr
{
  return TemplateEvent<LiftSessionBegin>::make(std::move(open));
}

//==============================================================================
auto Graph::Lane::Event::make(LiftSessionEnd close) -> EventPtr
{
  return TemplateEvent<LiftSessionEnd>::make(std::move(close));
}

//==============================================================================
auto Graph::Lane::Event::make(LiftMove move) -> EventPtr
{
  return TemplateEvent<LiftMove>::make(std::move(move));
}

//==============================================================================
auto Graph::Lane::Event::make(LiftDoorOpen open) -> EventPtr
{
  return TemplateEvent<LiftDoorOpen>::make(std::move(open));
}

//==============================================================================
auto Graph::Lane::Event::make(Dock dock) -> EventPtr
{
  return TemplateEvent<Dock>::make(std::move(dock));
}

//==============================================================================
auto Graph::Lane::Event::make(Wait wait) -> EventPtr
{
  return TemplateEvent<Wait>::make(std::move(wait));
}

//==============================================================================
class Graph::Lane::Node::Implementation
{
public:

  std::size_t waypoint;

  rmf_utils::clone_ptr<Event> _event;

  rmf_utils::clone_ptr<OrientationConstraint> _orientation;

};

//==============================================================================
Graph::Lane::Node::Node(
  std::size_t waypoint_index,
  rmf_utils::clone_ptr<Event> event,
  rmf_utils::clone_ptr<OrientationConstraint> orientation)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        waypoint_index,
        std::move(event),
        std::move(orientation)
      }))
{
  // Do nothing
}

//==============================================================================
Graph::Lane::Node::Node(
  std::size_t waypoint_index,
  rmf_utils::clone_ptr<OrientationConstraint> orientation)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        waypoint_index,
        nullptr,
        std::move(orientation)
      }))
{
  // Do nothing
}

//==============================================================================
std::size_t Graph::Lane::Node::waypoint_index() const
{
  return _pimpl->waypoint;
}

//==============================================================================
auto Graph::Lane::Node::event() const -> const Event*
{
  return _pimpl->_event.get();
}

//==============================================================================
Graph::Lane::Node& Graph::Lane::Node::event(
  rmf_utils::clone_ptr<Event> new_event)
{
  _pimpl->_event = std::move(new_event);
  return *this;
}

//==============================================================================
auto Graph::Lane::Node::orientation_constraint() const
-> const OrientationConstraint*
{
  return _pimpl->_orientation.get();
}

//==============================================================================
class Graph::Lane::Properties::Implementation
{
public:

  std::optional<double> speed_limit;
  std::string mutex_group;

};

//==============================================================================
Graph::Lane::Properties::Properties()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
std::optional<double> Graph::Lane::Properties::speed_limit() const
{
  return _pimpl->speed_limit;
}

//==============================================================================
auto Graph::Lane::Properties::speed_limit(std::optional<double> value)
-> Properties&
{
  _pimpl->speed_limit = value;
  return *this;
}

//==============================================================================
const std::string& Graph::Lane::Properties::in_mutex_group() const
{
  return _pimpl->mutex_group;
}

//==============================================================================
auto Graph::Lane::Properties::set_in_mutex_group(std::string group_name)
-> Properties&
{
  _pimpl->mutex_group = std::move(group_name);
  return *this;
}

//==============================================================================
class Graph::Lane::Implementation
{
public:

  std::size_t index;

  Node entry;

  Node exit;

  Properties properties;

  template<typename... Args>
  static Lane make(Args&& ... args)
  {
    Lane lane;
    lane._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{std::forward<Args>(args)...});

    return lane;
  }
};

//==============================================================================
auto Graph::Lane::entry() -> Node&
{
  return _pimpl->entry;
}

//==============================================================================
auto Graph::Lane::entry() const -> const Node&
{
  return _pimpl->entry;
}

//==============================================================================
auto Graph::Lane::exit() -> Node&
{
  return _pimpl->exit;
}

//==============================================================================
auto Graph::Lane::exit() const -> const Node&
{
  return _pimpl->exit;
}

//==============================================================================
auto Graph::Lane::properties() -> Properties&
{
  return _pimpl->properties;
}

//==============================================================================
auto Graph::Lane::properties() const -> const Properties&
{
  return _pimpl->properties;
}

//==============================================================================
std::size_t Graph::Lane::index() const
{
  return _pimpl->index;
}

//==============================================================================
Graph::Lane::Lane()
{
  // Do nothing
}

//==============================================================================
Graph::Graph()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
auto Graph::add_waypoint(
  std::string map_name,
  Eigen::Vector2d location) -> Waypoint&
{
  _pimpl->waypoints.emplace_back(
    Waypoint::Implementation::make(
      _pimpl->waypoints.size(),
      std::move(map_name), std::move(location)));

  _pimpl->lanes_from.push_back({});
  _pimpl->lanes_into.push_back({});
  _pimpl->lane_between.push_back({});

  return _pimpl->waypoints.back();
}

//==============================================================================
auto Graph::get_waypoint(const std::size_t index) -> Waypoint&
{
  return _pimpl->waypoints.at(index);
}

//==============================================================================
auto Graph::get_waypoint(const std::size_t index) const -> const Waypoint&
{
  return _pimpl->waypoints.at(index);
}

//==============================================================================
auto Graph::find_waypoint(const std::string& key) -> Waypoint*
{
  const auto it = _pimpl->keys.find(key);
  if (it == _pimpl->keys.end())
    return nullptr;

  return &get_waypoint(it->second);
}

//==============================================================================
auto Graph::find_waypoint(const std::string& key) const -> const Waypoint*
{
  return const_cast<Graph&>(*this).find_waypoint(key);
}

//==============================================================================
bool Graph::add_key(const std::string& key, std::size_t wp_index)
{
  if (wp_index > _pimpl->waypoints.size())
    return false;

  const auto inserted = _pimpl->keys.insert({key, wp_index}).second;
  if (!inserted)
    return false;

  Waypoint::Implementation::get(_pimpl->waypoints.at(wp_index)).name = key;
  return true;
}

//==============================================================================
bool Graph::remove_key(const std::string& key)
{
  const auto it = _pimpl->keys.find(key);
  if (it == _pimpl->keys.end())
    return false;

  Waypoint::Implementation::get(_pimpl->waypoints.at(it->second))
  .name = rmf_utils::nullopt;

  _pimpl->keys.erase(it);
  return true;
}

//==============================================================================
bool Graph::set_key(const std::string& key, std::size_t wp_index)
{
  if (_pimpl->waypoints.size() <= wp_index)
    return false;

  _pimpl->keys[key] = wp_index;
  const auto insertion = _pimpl->keys.insert({key, wp_index});
  if (!insertion.second)
  {
    Waypoint::Implementation::get(
      _pimpl->waypoints.at(insertion.first->second)).name = rmf_utils::nullopt;
    insertion.first->second = wp_index;
  }

  Waypoint::Implementation::get(_pimpl->waypoints.at(wp_index)).name = key;
  return true;
}

//==============================================================================
const std::unordered_map<std::string, std::size_t>& Graph::keys() const
{
  return _pimpl->keys;
}

//==============================================================================
std::size_t Graph::num_waypoints() const
{
  return _pimpl->waypoints.size();
}

//==============================================================================
auto Graph::add_lane(
  const Lane::Node& entry,
  const Lane::Node& exit,
  Lane::Properties properties) -> Lane&
{
  assert(entry.waypoint_index() < _pimpl->waypoints.size());
  assert(exit.waypoint_index() < _pimpl->waypoints.size());

  const std::size_t lane_id = _pimpl->lanes.size();
  const std::size_t entry_index = entry.waypoint_index();
  _pimpl->lanes_from.at(entry_index).push_back(lane_id);
  _pimpl->lanes_into.at(exit.waypoint_index()).push_back(lane_id);
  _pimpl->lane_between.at(entry_index)[exit.waypoint_index()] = lane_id;

  _pimpl->lanes.emplace_back(
    Lane::Implementation::make(
      _pimpl->lanes.size(),
      std::move(entry),
      std::move(exit),
      std::move(properties)));

  return _pimpl->lanes.back();
}

//==============================================================================
auto Graph::get_lane(const std::size_t index) -> Lane&
{
  return _pimpl->lanes.at(index);
}

//==============================================================================
auto Graph::get_lane(const std::size_t index) const -> const Lane&
{
  return _pimpl->lanes.at(index);
}

//==============================================================================
std::size_t Graph::num_lanes() const
{
  return _pimpl->lanes.size();
}

//==============================================================================
const std::vector<std::size_t>& Graph::lanes_from(std::size_t wp_index) const
{
  return _pimpl->lanes_from.at(wp_index);
}

//==============================================================================
const std::vector<std::size_t>& Graph::lanes_into(std::size_t wp_index) const
{
  return _pimpl->lanes_into.at(wp_index);
}

//==============================================================================
auto Graph::lane_from(std::size_t from_wp, std::size_t to_wp) -> Lane*
{
  const auto& lanes = _pimpl->lane_between.at(from_wp);
  const auto it = lanes.find(to_wp);
  if (it == lanes.end())
    return nullptr;

  return &_pimpl->lanes.at(it->second);
}

//==============================================================================
auto Graph::lane_from(std::size_t from_wp, std::size_t to_wp) const
-> const Lane*
{
  return const_cast<Graph&>(*this).lane_from(from_wp, to_wp);
}

//==============================================================================
auto Graph::set_known_lift(LiftProperties lift) -> LiftPropertiesPtr
{
  const auto [l_it, inserted] = _pimpl->lifts.insert({lift.name(), nullptr});
  if (inserted)
  {
    l_it->second = std::make_shared<LiftProperties>(std::move(lift));
  }
  else
  {
    *l_it->second = std::move(lift);
  }

  return l_it->second;
}

//==============================================================================
auto Graph::all_known_lifts() const -> std::vector<LiftPropertiesPtr>
{
  std::vector<LiftPropertiesPtr> lifts;
  lifts.reserve(_pimpl->lifts.size());
  for (const auto& [_, lift] : _pimpl->lifts)
  {
    lifts.push_back(lift);
  }

  return lifts;
}

//==============================================================================
auto Graph::find_known_lift(const std::string& name) const -> LiftPropertiesPtr
{
  const auto l_it = _pimpl->lifts.find(name);
  if (l_it == _pimpl->lifts.end())
    return nullptr;

  return l_it->second;
}

//==============================================================================
auto Graph::set_known_door(DoorProperties door) -> DoorPropertiesPtr
{
  const auto [d_it, inserted] = _pimpl->doors.insert({door.name(), nullptr});
  if (inserted)
  {
    d_it->second = std::make_shared<DoorProperties>(std::move(door));
  }
  else
  {
    *d_it->second = std::move(door);
  }

  return d_it->second;
}

//==============================================================================
auto Graph::all_known_doors() const -> std::vector<DoorPropertiesPtr>
{
  std::vector<DoorPropertiesPtr> doors;
  doors.reserve(_pimpl->doors.size());
  for (const auto& [_, door] : _pimpl->doors)
  {
    doors.push_back(door);
  }

  return doors;
}

//==============================================================================
auto Graph::find_known_door(const std::string& name) const -> DoorPropertiesPtr
{
  const auto d_it = _pimpl->doors.find(name);
  if (d_it == _pimpl->doors.end())
    return nullptr;

  return d_it->second;
}

} // namespace avg
} // namespace rmf_traffic
