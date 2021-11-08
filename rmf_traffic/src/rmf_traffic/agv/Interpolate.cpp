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

#include <rmf_traffic/agv/Interpolate.hpp>

#include "internal_Interpolate.hpp"

#include <rmf_utils/math.hpp>

namespace rmf_traffic {
namespace agv {

namespace {

//==============================================================================
struct State
{
  // Position
  double s;

  // Velocity
  double v;

  // Time
  Time t;

  State(double s_in, double v_in, double t_in, Time t_start)
  : s(s_in),
    v(v_in),
    t(t_start +
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(t_in)))
  {
    // Do nothing
  }
};

using States = std::vector<State>;

//==============================================================================
States compute_traversal(
  const Time start_time,
  const double s_f,
  const double v_nom,
  const double a_nom)
{
  States states;
  states.reserve(3);

  // Time spent accelerating
  const double t_a = std::min(std::sqrt(s_f/a_nom), v_nom/a_nom);

  // Position and velocity at the end of accelerating
  const double s_a = 0.5*a_nom*pow(t_a, 2);
  const double v = a_nom * t_a;
  states.emplace_back(s_a, v, t_a, start_time);

  // Time to begin decelerating
  const double t_d = s_f/v - s_a/v - 0.5*v/a_nom + t_a;

  if (t_d - t_a > 0)
  {
    const double s_d = v*(t_d - t_a) + s_a;
    states.emplace_back(s_d, v, t_d, start_time);
  }

  const double t_f = v/a_nom + t_d;
  states.emplace_back(s_f, 0.0, t_f, start_time);

  return states;
}

} // anonymous namespace

namespace internal {
//==============================================================================
void interpolate_translation(
  Trajectory& trajectory,
  const double v_nom,
  const double a_nom,
  const Time start_time,
  const Eigen::Vector3d& start,
  const Eigen::Vector3d& finish,
  const double threshold)
{
  const double heading = start[2];
  const Eigen::Vector2d start_p = start.block<2, 1>(0, 0);
  const Eigen::Vector2d finish_p = finish.block<2, 1>(0, 0);
  const Eigen::Vector2d diff_p = finish_p - start_p;
  const double dist = diff_p.norm();
  if (dist < threshold)
    return;

  const Eigen::Vector2d dir = diff_p/dist;

  // This function assumes that the initial position is already inside of the
  // trajectory.
  States states = compute_traversal(start_time, dist, v_nom, a_nom);
  for (const State& state : states)
  {
    const Eigen::Vector2d p_s = dir * state.s + start_p;
    const Eigen::Vector2d v_s = dir * state.v;

    const Eigen::Vector3d p{p_s[0], p_s[1], heading};
    const Eigen::Vector3d v{v_s[0], v_s[1], 0.0};
    trajectory.insert(state.t, p, v);
  }
}

//==============================================================================
Duration estimate_rotation_time(
  const double w_nom,
  const double alpha_nom,
  const double start_heading,
  const double finish_heading,
  const double threshold)
{
  const double diff_heading =
    rmf_utils::wrap_to_pi(finish_heading - start_heading);

  const double diff_heading_abs = std::abs(diff_heading);
  if (diff_heading_abs < threshold)
    return Duration(0);

  const auto start_time = Time(Duration(0));
  States states = compute_traversal(
    start_time, diff_heading_abs, w_nom, alpha_nom);

  return states.back().t - start_time;
}

//==============================================================================
bool interpolate_rotation(
  Trajectory& trajectory,
  const double w_nom,
  const double alpha_nom,
  const Time start_time,
  const Eigen::Vector3d& start,
  const Eigen::Vector3d& finish,
  const double threshold)
{
  const double start_heading = start[2];
  const double finish_heading = finish[2];
  const double diff_heading =
    rmf_utils::wrap_to_pi(finish_heading - start_heading);

  const double diff_heading_abs = std::abs(diff_heading);
  if (diff_heading_abs < threshold)
    return false;

  const double dir = diff_heading < 0.0 ? -1.0 : 1.0;

  States states = compute_traversal(
    start_time, diff_heading_abs, w_nom, alpha_nom);

  for (const State& state : states)
  {
    const double s = rmf_utils::wrap_to_pi(start_heading + dir*state.s);
    const double w = dir*state.v;

    //printf("s: %g\n", s);

    const Eigen::Vector3d p{finish[0], finish[1], s};
    const Eigen::Vector3d v{0.0, 0.0, w};
    trajectory.insert(state.t, p, v);
  }

  return true;
}

void circular_subarc_to_trajectory(Eigen::Vector2d circle_center, 
    double start_arc_radians, double end_arc_radians,
    rmf_traffic::Trajectory& traj, 
    std::chrono::time_point<std::chrono::steady_clock> start_time,
    std::chrono::time_point<std::chrono::steady_clock>& end_time_out,
    double velocity)
{
  // https://mechanicalexpressions.com/explore/geometric-modeling/circle-spline-approximation.pdf
  // form a control polygon with the start,midpoint,endpoint of the arc,
  // and convert it to hermite spline parameters
  Eigen::Vector3d start_arc_vector = Eigen::Vector3d(cos(start_arc_radians), sin(start_arc_radians), 0.0) * turning_radius;
  Eigen::Vector3d end_arc_vector = Eigen::Vector3d(cos(end_arc_radians), sin(end_arc_radians), 0.0) * turning_radius;

  Eigen::Vector3d p0 = Eigen::Vector3d(circle_center.x(), circle_center.y(), 0.0) + start_arc_vector;
  Eigen::Vector3d p3 = Eigen::Vector3d(circle_center.x(), circle_center.y(), 0.0) + end_arc_vector;

  double center_yaw = start_arc_radians + (end_arc_radians - start_arc_radians) * 0.5;
  Eigen::Vector2d midpoint_vector = Eigen::Vector2d(cos(center_yaw), sin(center_yaw)) * turning_radius;
  //IMDraw::draw_circle(start_circle_pos + midpoint_vector, 0.05, sf::Color(0, 127,0));
  Eigen::Vector2d midpoint = circle_center + midpoint_vector;

  Eigen::Vector3d p0_direction = Eigen::Vector3d(start_arc_vector.y(), -start_arc_vector.x(), 0.0);
  Eigen::Vector3d p3_direction = Eigen::Vector3d(-end_arc_vector.y(), end_arc_vector.x(), 0.0);
  p0_direction.normalize();
  p3_direction.normalize();

  double k = (midpoint.x() - 0.5 * p0.x() - 0.5 * p3.x()) / (0.375 * (p0_direction.x() + p3_direction.x()));
  //printf("k: %g\n", k);
  
  Eigen::Vector3d p1 = p0 + p0_direction * k;
  Eigen::Vector3d p2 = p3 + p3_direction * k;

  // estimate duration
  double arclength = std::abs(end_arc_radians - start_arc_radians) * turning_radius;
  double arc_duration = arclength / velocity;
  // printf("start_arc_radians %g end_arc_radians %g arc_duration: %g\n", start_arc_radians, end_arc_radians, arc_duration);
  end_time_out = start_time + rmf_traffic::time::from_seconds(arc_duration);
  double duration_per_slice = arc_duration / 3.0;

  auto convert_bspline_to_hermite = [&](Eigen::Vector3d p0, Eigen::Vector3d p1,
      Eigen::Vector3d p2, Eigen::Vector3d p3, sf::Color c, double offset_timing)
  {
    Eigen::Matrix4d mtx_bspline;
    mtx_bspline << 
      -1.0,  3.0, -3.0,  1.0,
        3.0, -6.0,  3.0,  0.0,
      -3.0,  0.0,  3.0,  0.0,
        1.0,  4.0,  1.0,  0.0;
    mtx_bspline /= 6.0;
    
    Eigen::Matrix4d mtx_hermite;
    mtx_hermite << 
      2.0, -2.0, 1.0, 1.0,
      -3.0, 3.0, -2.0, -1.0,
      0.0, 0.0, 1.0, 0.0,
      1.0, 0.0, 0.0, 0.0;

    Eigen::Matrix4d mtx_hermite_inv = mtx_hermite.inverse();

    Eigen::Matrix<double, 4, 3> input;
    input << 
      p0[0], p0[1], p0[2],
      p1[0], p1[1], p1[2],
      p2[0], p2[1], p2[2],
      p3[0], p3[1], p3[2];

    auto result = mtx_hermite_inv * mtx_bspline * input;
    Eigen::Vector2d hermite_p0(result(0, 0), result(0, 1));
    Eigen::Vector2d hermite_p1(result(1, 0), result(1, 1));
    Eigen::Vector2d hermite_v0(result(2, 0), result(2, 1));
    Eigen::Vector2d hermite_v1(result(3, 0), result(3, 1));

    auto start_t = start_time + rmf_traffic::time::from_seconds(offset_timing);
    auto end_time = start_t + rmf_traffic::time::from_seconds(duration_per_slice);

    Eigen::Vector3d hermite_p0_3d(result(0, 0), result(0, 1), result(0, 2));
    Eigen::Vector3d hermite_p1_3d(result(1, 0), result(1, 1), result(1, 2));
    Eigen::Vector3d hermite_v0_3d(result(2, 0), result(2, 1), result(2, 2));
    Eigen::Vector3d hermite_v1_3d(result(3, 0), result(3, 1), result(3, 2));

    if (!traj.empty())
    {
      // WORKAROUND
      // sometimes the bspline cuts off earlier at its tips than it should.
      // could be the division by 6.0 that fcl uses in its basis matrix
      // so we make sure the points join up
      auto& last_waypoint = traj.back();
      last_waypoint.position(hermite_p0_3d);
      last_waypoint.velocity(hermite_v0_3d / duration_per_slice);
    }

    traj.insert(start_t, hermite_p0_3d, hermite_v0_3d / duration_per_slice);
    traj.insert(end_time, hermite_p1_3d, hermite_v1_3d / duration_per_slice);
  };

  sf::Color color(0, 127, 0);
  convert_bspline_to_hermite(p0, p0, p1, p2, color, 0.0);
  convert_bspline_to_hermite(p0, p1, p2, p3, color, duration_per_slice);
  convert_bspline_to_hermite(p1, p2, p3, p3, color, duration_per_slice * 2.0);
}

void interpolate_circular_arc_trajectory(Eigen::Vector2d circle_center, 
    Eigen::Vector2d start_arc_pt, Eigen::Vector2d end_arc_pt,
    rmf_traffic::Trajectory& traj, 
    std::chrono::time_point<std::chrono::steady_clock> start_time,
    std::chrono::time_point<std::chrono::steady_clock>& end_time_out,
    double velocity, bool anticlockwise)
{
  Eigen::Vector2d start_arc_vector = start_arc_pt - circle_center;
  start_arc_vector.normalize();
  Eigen::Vector2d end_arc_vector = end_arc_pt - circle_center;
  end_arc_vector.normalize();

  double start_arc_radians = atan2(start_arc_vector.y(), start_arc_vector.x());
  double end_arc_radians = atan2(end_arc_vector.y(), end_arc_vector.x());

  // break into subarcs to better approximate the circular arc
  double arc_difference = end_arc_radians - start_arc_radians;
  if (!anticlockwise)
    arc_difference = -(2.0 * M_PI - arc_difference);

  double min_interval_rot = M_PI_2;
  int intervals = 1;
  if (arc_difference > min_interval_rot)
  {
    double interval_d = arc_difference / min_interval_rot;
    intervals = (int)std::ceil(interval_d);
  }

  double arc_per_interval = arc_difference / (double)intervals;
  for (int i=0; i<intervals; ++i)
  {
    double start_arc = start_arc_radians + arc_per_interval * (double)i;
    double end_arc = start_arc_radians + arc_per_interval * (double)(i + 1);

    if (i != 0)
      start_time = end_time_out;
    circular_subarc_to_trajectory(circle_center, start_arc, end_arc,
      traj, start_time, end_time_out, velocity);
  }
}

bool interpolate_ackermann_straightline_curve_to_next(
  Trajectory& trajectory,
  const double v_start,
  const double v_nom,
  const double a_nom,
  const double w_nom,
  const double alpha_nom,
  const Time start_time,
  const Eigen::Vector3d& start,
  const Eigen::Vector3d& mid,
  const Eigen::Vector3d& finish,
  const double turn_radius,
  const double threshold)
{
  // this function was intended for a turn in a tripleset of waypoints
  // and will ignore the yaw direction

  Eigen::Vector2d start2d(start.x(), start.y());
  Eigen::Vector2d mid2d(mid.x(), mid.y());
  Eigen::Vector2d finish2d(finish.x(), finish.y());

  Eigen::Vector2d start_to_mid = mid2d - start2d;
  start_to_mid.normalize();
  Eigen::Vector2d mid_to_finish = finish2d - mid2d;
  mid_to_finish.normalize();

  // figure out turning points, labelled p1 and p2
  Eigen::Vector2d p1;
  Eigen::Vector2d p2;

  // figure out circle center


  // add the straight line trajectory that start with speed v_start
  // and ends with v_nom
  

  // add the circular arc trajectory
  interpolate_circular_arc_trajectory(trajectory, );

  

}

} // namespace internal

//==============================================================================
class invalid_traits_error::Implementation
{
public:

  static invalid_traits_error make_error(
    const VehicleTraits& traits)
  {
    std::vector<std::pair<std::string, double>> values;
    for (const auto& pair :
      {std::make_pair("linear velocity",
        traits.linear().get_nominal_velocity()),
        std::make_pair("linear acceleration",
        traits.linear().get_nominal_acceleration()),
        std::make_pair("rotational velocity",
        traits.rotational().get_nominal_velocity()),
        std::make_pair("rotational acceleration",
        traits.rotational().get_nominal_acceleration())})
    {
      if (pair.second <= 0.0)
        values.push_back(pair);
    }

    assert(!values.empty());

    invalid_traits_error error;
    error._pimpl->_what =
      "[invalid_traits_error] The following traits have invalid values:";
    for (const auto& pair : values)
    {
      error._pimpl->_what +=
        "\n -- " + pair.first + ": " + std::to_string(pair.second);
    }

    return error;
  }

  std::string _what;
};

//==============================================================================
const char* invalid_traits_error::what() const noexcept
{
  return _pimpl->_what.c_str();
}

//==============================================================================
invalid_traits_error::invalid_traits_error()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
Interpolate::Options::Options(
  const bool always_stop,
  const double translation_thresh,
  const double rotation_thresh,
  const double corner_angle_thresh)
: _pimpl(rmf_utils::make_impl<Implementation>(
      always_stop,
      translation_thresh,
      rotation_thresh,
      corner_angle_thresh))
{
  // Do nothing
}

//==============================================================================
Interpolate::Options& Interpolate::Options::set_always_stop(bool choice)
{
  _pimpl->always_stop = choice;
  return *this;
}

//==============================================================================
bool Interpolate::Options::always_stop() const
{
  return _pimpl->always_stop;
}

//==============================================================================
Interpolate::Options& Interpolate::Options::set_translation_threshold(
  double dist)
{
  _pimpl->translation_thresh = dist;
  return *this;
}

//==============================================================================
double Interpolate::Options::get_translation_threshold() const
{
  return _pimpl->translation_thresh;
}

//==============================================================================
Interpolate::Options& Interpolate::Options::set_rotation_threshold(double angle)
{
  _pimpl->rotation_thresh = angle;
  return *this;
}

//==============================================================================
double Interpolate::Options::get_rotation_threshold() const
{
  return _pimpl->rotation_thresh;
}

//==============================================================================
Interpolate::Options& Interpolate::Options::set_corner_angle_threshold(
  double angle)
{
  _pimpl->corner_angle_thresh = angle;
  return *this;
}

//==============================================================================
double Interpolate::Options::get_corner_angle_threshold() const
{
  return _pimpl->corner_angle_thresh;
}

//==============================================================================
namespace internal {
bool can_skip_interpolation(
  const Eigen::Vector3d& last_position,
  const Eigen::Vector3d& next_position,
  const Eigen::Vector3d& future_position,
  const Interpolate::Options::Implementation& options)
{
  const Eigen::Vector2d next_p = next_position.block<2, 1>(0, 0);
  const Eigen::Vector2d last_p = last_position.block<2, 1>(0, 0);
  const Eigen::Vector2d future_p = future_position.block<2, 1>(0, 0);

  // If the waypoints are very close together, then we can skip it
  bool can_skip =
    (next_p - last_p).norm() < options.translation_thresh
    || (future_p - next_p).norm() < options.translation_thresh;

  // Check if the corner is too sharp
  const Eigen::Vector2d d_next_p = next_p - last_p;
  const Eigen::Vector2d d_future_p = future_p - next_p;
  const double d_next_p_norm = d_next_p.norm();
  const double d_future_p_norm = d_future_p.norm();

  if (d_next_p_norm > 1e-8 && d_future_p_norm > 1e-8)
  {
    const double dot_product = d_next_p.dot(d_future_p);
    const double angle = std::acos(
      dot_product/(d_next_p_norm * d_future_p_norm));
    // If the corner is smaller than the threshold, then we can skip it
    if (angle < options.corner_angle_thresh)
      can_skip = true;
  }

  const double next_angle = next_position[2];
  const double last_angle = last_position[2];
  const double future_angle = future_position[2];
  if (can_skip && (
      std::abs(next_angle - last_angle) > options.rotation_thresh
      || std::abs(future_angle - next_angle) > options.rotation_thresh))
  {
    can_skip = false;
  }

  return can_skip;
}

} // namespace internal

//==============================================================================
Trajectory Interpolate::positions(
  const VehicleTraits& traits,
  const Time start_time,
  const std::vector<Eigen::Vector3d>& input_positions,
  const Options& input_options)
{
  if (!traits.valid())
    throw invalid_traits_error::Implementation::make_error(traits);

  Trajectory trajectory;

  if (input_positions.empty())
    return trajectory;

  trajectory.insert(
    start_time,
    input_positions.front(),
    Eigen::Vector3d::Zero());
  assert(trajectory.size() > 0);
  printf("generate\n");

  const double v = traits.linear().get_nominal_velocity();
  const double a = traits.linear().get_nominal_acceleration();
  const double w = traits.rotational().get_nominal_velocity();
  const double alpha = traits.rotational().get_nominal_acceleration();
  const auto options = Options::Implementation::get(input_options);

  const std::size_t N = input_positions.size();
  std::size_t last_stop_index = 0;
  for (std::size_t i = 1; i < N; ++i)
  {
    const Eigen::Vector3d& last_position = input_positions[last_stop_index];
    const Eigen::Vector3d& next_position = input_positions[i];

    if (!options.always_stop && i+1 < N)
    {
      const Eigen::Vector3d& future_position = input_positions[i+1];
      if (internal::can_skip_interpolation(
          last_position, next_position, future_position, options))
      {
        continue;
      }
    }

    assert(trajectory.finish_time());
    internal::interpolate_translation(
      trajectory, v, a, *trajectory.finish_time(), last_position,
      next_position, options.translation_thresh);

    internal::interpolate_rotation(
      trajectory, w, alpha, *trajectory.finish_time(), last_position,
      next_position, options.rotation_thresh);

    last_stop_index = i;
  }

  return trajectory;
}

} // namespace agv
} // namespace rmf_traffic
