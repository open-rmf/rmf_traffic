
#include "CCD.hpp"
#include <Eigen/Dense>

namespace rmf_traffic {

//==============================================================================
inline double distsq_lineseg_to_pt(
  const Eigen::Vector3d (&pts)[4], int ls_idx_start, int ls_idx_end,
  const Eigen::Vector3d& pt, int* pt_on_lineseg_idx = nullptr)
{
  if (pt_on_lineseg_idx)
    *pt_on_lineseg_idx = -1; // not on either endpoint

  auto ls_vec = pts[ls_idx_end] - pts[ls_idx_start];
  double len = ls_vec.norm();
  auto ls_vec_norm = ls_vec / len;
  
  double l = ls_vec_norm.dot(pt - pts[ls_idx_start]);
  if (l <= 0.0)
  {
    l = 0.0;
    if (pt_on_lineseg_idx)
      *pt_on_lineseg_idx = ls_idx_start;
  }
  if (l >= len)
  {
    l = len;
    if (pt_on_lineseg_idx)
      *pt_on_lineseg_idx = ls_idx_end;
  }

  auto closest_pt_on_ls = pts[ls_idx_start] + l * ls_vec_norm;

  double distsq = (closest_pt_on_ls - pt).squaredNorm();
  return distsq;
}

//==============================================================================
inline double min_distsq_lineseg_to_lineseg(
  const Eigen::Vector3d& p1, const Eigen::Vector3d& q1,
  const Eigen::Vector3d& p2, const Eigen::Vector3d& q2,
  double& s1, double& s2)
{
  // From Real Time Collision Detection, chapter 5.1.9
  double s = 0.0, t = 0.0;

  auto d1 = q1 - p1;
  auto d2 = q2 - p2;
  auto r = p1 - p2;
  double a = d1.squaredNorm();
  double e = d2.squaredNorm();
  double f = d2.dot(r);

  double epsilon = 1e-06;
  if (a <= epsilon && e <= epsilon)
    return (p2 - p1).squaredNorm();
  if (a <= epsilon)
  {
    s = 0.0;
    t = f / e;
    fcl::clipToRange(t, 0.0, 1.0);
  }
  else
  {
    double c = d1.dot(r);
    if (e <= epsilon)
    {
      t = 0.0;
      s = -c / a;
      fcl::clipToRange(s, 0.0, 1.0);
    }
    else
    {
      double b = d1.dot(d2);
      double denom = a * e - b * b;
      if (denom != 0.0)
      {
        s = (b * f - c * e) / denom;
        fcl::clipToRange(s, 0.0, 1.0);
      }
      else
        s = 0.0;
      
      t = (b * s + f) / e;
      if (t < 0.0)
      {
        t = 0.0;
        s = -c /a;
        fcl::clipToRange(s, 0.0, 1.0);
      }
      else if (t > 1.0)
      {
        t = 1.0;
        s = (b - c) / a;
        fcl::clipToRange(s, 0.0, 1.0);
      }
    }
  }

  auto c1 = p1 + d1 * s;
  auto c2 = p2 + d2 * t;
  s1 = s;
  s2 = t;
  return (c2 - c1).squaredNorm();
}

//==============================================================================
static const int INSIDE = 0b0000;
static const int LEFT   = 0b0001;
static const int RIGHT  = 0b0010;
static const int BOTTOM = 0b0100;
static const int TOP    = 0b1000;

//==============================================================================
double sphere_box_closest_features(
  const fcl::Sphered& sphere,
  const fcl::Transform3d& tx_a,
  const fcl::Boxd& box,
  const fcl::Transform3d& tx_b,
  SeperationInfo& seperation_info)
{
  auto sphere_center = tx_a.translation();
  auto box_center = tx_b.translation();

  auto linear_b = tx_b.linear();
  auto col0_b = linear_b.col(0);
  auto col1_b = linear_b.col(1);

  double halfsize_x = box.side.x() * 0.5;
  double halfsize_y = box.side.y() * 0.5;

  auto box_to_sphere = sphere_center - box_center;
  double dotp1 = box_to_sphere.dot(col0_b);
  double dotp2 = box_to_sphere.dot(col1_b);

  int outcode = 0;
  if (dotp1 < -halfsize_x)
    outcode |= LEFT;
  if (dotp1 > halfsize_x)
    outcode |= RIGHT;
  if (dotp2 < -halfsize_y)
    outcode |= BOTTOM;
  if (dotp2 > halfsize_y)
    outcode |= TOP;
    
  if (outcode == INSIDE)
    return -1.0; //point inside box; collision

  // start bottom left move anticlockwise
  Eigen::Vector3d pts[4];
  pts[0] = box_center - col0_b * halfsize_x - col1_b * halfsize_y;
  pts[1] = box_center + col0_b * halfsize_x - col1_b * halfsize_y;
  pts[2] = box_center + col0_b * halfsize_x + col1_b * halfsize_y;
  pts[3] = box_center - col0_b * halfsize_x + col1_b * halfsize_y;
  double radius_sq = sphere.radius * sphere.radius;

  if (outcode == (LEFT | BOTTOM))
  {
    auto boxpt_to_sphere = sphere_center - pts[0];
    double distsq = boxpt_to_sphere.squaredNorm();
    if (distsq <= radius_sq)
      return -1.0; //circle intersecting box; collision

    // pick 2 points to be the closest features
    seperation_info.count_b = 1;
    seperation_info.pointindices_b[0] = 0;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }
  if (outcode == (RIGHT | BOTTOM))
  {
    auto boxpt_to_sphere = sphere_center - pts[1];
    double distsq = boxpt_to_sphere.squaredNorm();
    if (distsq <= radius_sq)
      return -1.0; //circle intersecting box; collision

    // pick 2 points to be the closest features
    seperation_info.count_b = 1;
    seperation_info.pointindices_b[0] = 1;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }
  if (outcode == (RIGHT | TOP))
  {
    auto boxpt_to_sphere = sphere_center - pts[2];
    double distsq = boxpt_to_sphere.squaredNorm();
    if (distsq <= radius_sq)
      return -1.0; //circle intersecting box; collision
    
    // pick points to be the closest features
    seperation_info.count_b = 1;
    seperation_info.pointindices_b[0] = 2;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }
  if (outcode == (LEFT | TOP))
  {
    auto boxpt_to_sphere = sphere_center - pts[3];
    double distsq = boxpt_to_sphere.squaredNorm();
    if (distsq <= radius_sq)
      return -1.0; //circle intersecting box; collision

    seperation_info.count_b = 1;
    seperation_info.pointindices_b[0] = 3;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;
    return std::sqrt(distsq);
  }

  // singular sides, compare with line segments
  if (outcode == LEFT)
  {
    double distsq = distsq_lineseg_to_pt(pts, 0, 3, sphere_center);
    if (distsq <= radius_sq)
      return -1.0; //intersection

    seperation_info.count_b = 2;
    seperation_info.pointindices_b[0] = 0;
    seperation_info.pointindices_b[1] = 3;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }

  if (outcode == RIGHT)
  {
    double distsq = distsq_lineseg_to_pt(pts, 2, 1, sphere_center);
    if (distsq <= radius_sq)
      return -1.0; //intersection

    seperation_info.count_b = 2;
    seperation_info.pointindices_b[0] = 2;
    seperation_info.pointindices_b[1] = 1;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }

  if (outcode == BOTTOM)
  {
    double distsq = distsq_lineseg_to_pt(pts, 0, 1, sphere_center);
    if (distsq <= radius_sq)
      return -1.0; //intersection

    seperation_info.count_b = 2;
    seperation_info.pointindices_b[0] = 0;
    seperation_info.pointindices_b[1] = 1;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }

  if (outcode == TOP)
  {
    double distsq = distsq_lineseg_to_pt(pts, 3, 2, sphere_center);
    if (distsq <= radius_sq)
      return -1.0; //intersection

    // pick the line segment's 2 points to be closest features
    seperation_info.count_b = 2;
    seperation_info.pointindices_b[0] = 3;
    seperation_info.pointindices_b[1] = 2;

    seperation_info.count_a = 1;
    seperation_info.pointindices_a[0] = 0;

    return std::sqrt(distsq);
  }
  
  return -1.0;
}

//==============================================================================
double box_box_closest_features(
  const fcl::Boxd& box_a,
  const fcl::Transform3d& tx_a,
  const fcl::Boxd& box_b,
  const fcl::Transform3d& tx_b,
  SeperationInfo& seperation_info)
{
  double halfsize_x_a = box_a.side.x() * 0.5;
  double halfsize_y_a = box_a.side.y() * 0.5;
  double halfsize_x_b = box_b.side.x() * 0.5;
  double halfsize_y_b = box_b.side.y() * 0.5;

  auto linear_a = tx_a.linear();
  auto col0_a = linear_a.col(0);
  auto col1_a = linear_a.col(1);

  auto linear_b = tx_b.linear();
  auto col0_b = linear_b.col(0);
  auto col1_b = linear_b.col(1);

  auto center_a = tx_a.translation();
  auto center_b = tx_b.translation();

  // form points of box a and b
  Eigen::Vector3d pts_a[4];
  pts_a[0] = center_a - col0_a * halfsize_x_a - col1_a * halfsize_y_a;
  pts_a[1] = center_a + col0_a * halfsize_x_a - col1_a * halfsize_y_a;
  pts_a[2] = center_a + col0_a * halfsize_x_a + col1_a * halfsize_y_a;
  pts_a[3] = center_a - col0_a * halfsize_x_a + col1_a * halfsize_y_a;

  Eigen::Vector3d pts_b[4];
  pts_b[0] = center_b - col0_b * halfsize_x_b - col1_b * halfsize_y_b;
  pts_b[1] = center_b + col0_b * halfsize_x_b - col1_b * halfsize_y_b;
  pts_b[2] = center_b + col0_b * halfsize_x_b + col1_b * halfsize_y_b;
  pts_b[3] = center_b - col0_b * halfsize_x_b + col1_b * halfsize_y_b;

  auto closest_box_features_outside_box_face = [](
    Eigen::Vector3d (&pts)[4], const Eigen::Vector3d& center,
    double halfside_x, double halfside_y,
    const Eigen::Vector3d& projection_x, const Eigen::Vector3d& projection_y,
    int& face_idx_1, int& face_idx_2,
    int& closest_idx_1, int& closest_idx_2)
      -> bool
  { 
    face_idx_1 = -1;
    face_idx_2 = -1;
    closest_idx_1 = -1;
    closest_idx_2 = -1;

    uint pts_left = 0, pts_right = 0, pts_top = 0, pts_bottom = 0;
    Eigen::Vector2d dotp[4];
    for (int i=0; i<4; ++i)
    {
      auto center_to_boxpt = pts[i] - center;
      dotp[i].x() = projection_x.dot(center_to_boxpt);
      dotp[i].y() = projection_y.dot(center_to_boxpt);

      if (dotp[i].x() > halfside_x)
        ++pts_right;
      else if (dotp[i].x() < -halfside_x)
        ++pts_left;

      if (dotp[i].y() > halfside_y)
        ++pts_top;
      else if (dotp[i].y() < -halfside_y)
        ++pts_bottom;
    }

    if (pts_left == 4 || pts_right == 4 || pts_top == 4 || pts_bottom == 4)
    {
      // get the minimum 2 point indices
      double distsq[4];
      for (int i=0; i<4; ++i)
        distsq[i] = dotp[i].squaredNorm();

      closest_idx_1 = 0;
      closest_idx_2 = 1;
      if (distsq[0] > distsq[1])
        std::swap(closest_idx_1, closest_idx_2);
      
      for (int i=2; i<4; ++i)
      { 
        if (distsq[i] < distsq[closest_idx_2])
        {
          if (distsq[i] < distsq[closest_idx_1])
          {
            closest_idx_2 = closest_idx_1;
            closest_idx_1 = i;
          }
          else
            closest_idx_2 = i;
        }
      }
      
      if (pts_left == 4)
      {
        face_idx_1 = 0;
        face_idx_2 = 3;
        return LEFT;
      }
      if (pts_right == 4)
      {
        face_idx_1 = 2;
        face_idx_2 = 1;
        return RIGHT;
      }
      if (pts_bottom == 4)
      {
        face_idx_1 = 0;
        face_idx_2 = 1;
        return BOTTOM;
      }
      if (pts_top == 4)
      {
        face_idx_1 = 2;
        face_idx_2 = 3;
        return TOP;
      }
    }
    
    return 0;
  };

  int face_a_1 = -1;
  int face_a_2 = -1;
  int closest_b_1 = -1;
  int closest_b_2 = -1;
  if (closest_box_features_outside_box_face(pts_b, center_a, 
    halfsize_x_a, halfsize_y_a, col0_a, col1_a, 
    face_a_1, face_a_2, closest_b_1, closest_b_2) != INSIDE)
  {
    double param_a = 0.0;
    double param_b = 0.0;
    double distsq = min_distsq_lineseg_to_lineseg(pts_a[face_a_1], pts_a[face_a_2], 
      pts_b[closest_b_1], pts_b[closest_b_2], param_a, param_b);

    seperation_info.set_indices_from_lineseg(param_a, face_a_1, face_a_2);
    seperation_info.set_indices_from_lineseg(param_b, closest_b_1, closest_b_2, false);
    return std::sqrt(distsq);
  }

  int face_b_1 = -1;
  int face_b_2 = -1;
  int closest_a_1 = -1;
  int closest_a_2 = -1;
  if (closest_box_features_outside_box_face(pts_a, center_b, 
    halfsize_x_b, halfsize_y_b, col0_b, col1_b, 
    face_b_1, face_b_2, closest_a_1, closest_a_2) != INSIDE)
  {
    double param_a = 0.0;
    double param_b = 0.0;
    double distsq = min_distsq_lineseg_to_lineseg(pts_b[face_b_1], pts_b[face_b_2],
      pts_a[closest_a_1], pts_a[closest_a_2], param_b, param_a);

    seperation_info.set_indices_from_lineseg(param_a, closest_a_1, closest_a_2);
    seperation_info.set_indices_from_lineseg(param_b, face_b_1, face_b_2, false);
    
    return std::sqrt(distsq);
  }

  return -1.0;
}

} // namespace rmf_traffic