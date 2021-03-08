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

#ifndef SRC__CCD_HPP
#define SRC__CCD_HPP

#include <algorithm>
#include <rmf_traffic/geometry/ConvexShape.hpp>

#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/math/motion/spline_motion.h>

namespace rmf_traffic {

struct SeperationInfo
{
  // closest features
  int count_a = 0;
  int count_b = 0;
  int pointindices_a[2] = { -1, -1 };
  int pointindices_b[2] = { -1, -1 };

  void set_indices_from_lineseg(double parameter, int ls_start_idx, int ls_end_idx, 
    bool use_a = true)
  {
    if (use_a)
    {
      if (parameter == 0.0)
      {
        count_a = 1;
        pointindices_a[0] = ls_start_idx;
      }
      else if (parameter == 1.0)
      {
        count_a = 1;
        pointindices_a[0] = ls_end_idx;
      }
      else
      {
        count_a = 2;
        pointindices_a[0] = ls_start_idx;
        pointindices_a[1] = ls_end_idx;
      }
    }
    else
    {
      if (parameter == 0.0)
      {
        count_b = 1;
        pointindices_b[0] = ls_start_idx;
      }
      else if (parameter == 1.0)
      {
        count_b = 1;
        pointindices_b[0] = ls_end_idx;
      }
      else
      {
        count_b = 2;
        pointindices_b[0] = ls_start_idx;
        pointindices_b[1] = ls_end_idx;
      }
    }
  }

  void swap()
  {
    for (int i=0; i<2; ++i)
      std::swap(pointindices_a[i], pointindices_b[i]);
    std::swap(count_a, count_b);
  }
};


double sphere_box_closest_features(
  const fcl::Sphered& sphere,
  const fcl::Transform3d& tx_a,
  const fcl::Boxd& box,
  const fcl::Transform3d& tx_b,
  SeperationInfo& seperation_info);

double box_box_closest_features(
  const fcl::Boxd& box_a,
  const fcl::Transform3d& tx_a,
  const fcl::Boxd& box_b,
  const fcl::Transform3d& tx_b,
  SeperationInfo& seperation_info);

bool collide_seperable_shapes(
  std::shared_ptr<fcl::MotionBased> motion_a,
  std::shared_ptr<fcl::MotionBased> motion_b,
  uint sweeps,
  const geometry::ConstShapeGroup& a_shapes,
  const geometry::ConstShapeGroup& b_shapes,
  double& impact_time, uint& iterations, 
  uint safety_maximum_iterations = 120,
  double tolerance = 0.001);

uint get_sweep_divisions(std::shared_ptr<fcl::MotionBased> motion_a,
  std::shared_ptr<fcl::MotionBased> motion_b);

} // namespace rmf_traffic

#endif // SRC__CCD_HPP