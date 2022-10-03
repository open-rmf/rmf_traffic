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

    const Eigen::Vector3d p{finish[0], finish[1], s};
    const Eigen::Vector3d v{0.0, 0.0, w};
    trajectory.insert(state.t, p, v);
  }

  return true;
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

//==============================================================================
TimeVelocity interpolate_time_along_quadratic_straight_line(
  const Trajectory& trajectory,
  const Eigen::Vector2d& p,
  double holding_point_tolerance)
{
  if (trajectory.size() < 2)
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[rmf_traffic::agv::internal::"
      "interpolate_time_along_quadratic_straight_line] "
      "Invalid size for trajectory: " + std::to_string(trajectory.size()));
    // INDENT-ON*
  }

  const Eigen::Vector2d p0 = trajectory.front().position().block<2, 1>(0, 0);
  const Eigen::Vector2d p1 = trajectory.back().position().block<2, 1>(0, 0);
  const Eigen::Vector2d n = (p1 - p0).normalized();
  const double s = n.dot(p - p0);

  const auto convert = [](
    const double tau,
    const double t0,
    const double v0,
    const double a,
    const Eigen::Vector2d n) -> TimeVelocity
    {
      return {Time(time::from_seconds(tau + t0)), n*(v0 + a*tau)};
    };

  const auto choose = [convert](
      const double tau,
      const double tau_f,
      const double t0,
      const double v0,
      const double a,
      const Eigen::Vector2d n) -> std::optional<TimeVelocity>
    {
      if (tau_f + 1e-3 < tau)
        return std::nullopt;

      if (tau < -1e-3)
        return std::nullopt;

      // If tau is greater than tau_f but within the acceptable threshold, then
      // clamp it to tau_f.
      if (tau_f < tau)
        return convert(tau_f, t0, v0, a, n);

      // If tau is less than zero but within the acceptable threshold, then
      // clamp it to zero.
      if (tau < 0.0)
        return convert(0.0, t0, v0, a, n);

      return convert(tau, t0, v0, a, n);
    };

  for (std::size_t i = 1; i < trajectory.size(); ++i)
  {
    const auto& wp0 = trajectory[i-1];
    const auto& wp1 = trajectory[i];
    const double s0 = n.dot(wp0.position().block<2, 1>(0, 0) - p0);
    const double v0 = n.dot(wp0.velocity().block<2, 1>(0, 0));
    const double sf = n.dot(wp1.position().block<2, 1>(0, 0) - p0);
    const double t0 = time::to_seconds(wp0.time().time_since_epoch());
    const double tau_f = time::to_seconds(wp1.time().time_since_epoch()) - t0;

    const double a = 2*(sf-s0-v0*tau_f)/pow(tau_f, 2);
    const double radicand = v0*v0 + 2*a*(s-s0);
    if (std::abs(a) > 1e-16 && radicand >= 0.0)
    {
      const double tau_m = (-v0 - std::sqrt(radicand))/a;
      if (auto result = choose(tau_m, tau_f, t0, v0, a, n))
      {
        return *result;
      }

      const double tau_p = (-v0 + std::sqrt(radicand))/a;
      if (auto result = choose(tau_p, tau_f, t0, v0, a, n))
      {
        return *result;
      }
    }
    else if (std::abs(v0) > 1e-6)
    {
      const double tau = (s - s0)/v0;
      if (auto result = choose(tau, tau_f, t0, v0, a, n))
      {
        return *result;
      }
    }
    else if (std::abs(s - s0) < holding_point_tolerance)
    {
      return TimeVelocity{
        Time(time::from_seconds(t0)),
        Eigen::Vector2d::Zero()
      };
    }
  }

  std::stringstream ss;
  ss << "[rmf_traffic::agv::internal::"
     << "interpolate_time_along_quadratic_straight_line] Position ("
     << p.block<2, 1>(0, 0).transpose() << ") does not lie along the "
     << "trajectory:";

  for (const auto& wp : trajectory)
    ss << " (" << wp.position().block<2, 1>(0, 0).transpose() << ")";

  throw std::runtime_error(ss.str());
}

} // namespace agv
} // namespace rmf_traffic
