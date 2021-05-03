
#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/math/motion/spline_motion.h>

#include "Spline.hpp"
#include "CCD.hpp"
#include "geometry/ShapeInternal.hpp"
#include <float.h>

namespace rmf_traffic {

//==============================================================================
struct SeperationComputation
{
  int support_vertex_a = -1;
  int support_vertex_b = -1;

  int count_a = 0;
  int count_b = 0;
  Eigen::Vector3d local_points_a[4];
  Eigen::Vector3d local_points_b[4];

  Eigen::Vector3d seperation_axis;
  Eigen::Vector3d local_axis_point;

  const SeperationInfo initial_seperation_props;

  SeperationComputation(
    std::shared_ptr<fcl::MotionBased> motion_a,
    std::shared_ptr<fcl::MotionBased> motion_b,
    const geometry::ConstFinalShapePtr& shape_a,
    const geometry::ConstFinalShapePtr& shape_b,
    const SeperationInfo& seperation_info)
    :initial_seperation_props(seperation_info)
  {
    auto update_locals = [](const geometry::CollisionGeometryPtr& geom,
      int& count, Eigen::Vector3d (&local_points)[4])
    {
      if (geom->getNodeType() == fcl::GEOM_SPHERE)
      {
        count = 1;
        local_points[0].setZero();
      }
      else if (geom->getNodeType() == fcl::GEOM_BOX)
      {
        auto box =
          std::dynamic_pointer_cast<fcl::Boxd>(geom);
        count = 4;
        auto halfside = box->side * 0.5;
        local_points[0] = Eigen::Vector3d(-halfside.x(), -halfside.y(), 0.0);
        local_points[1] = Eigen::Vector3d( halfside.x(), -halfside.y(), 0.0);
        local_points[2] = Eigen::Vector3d( halfside.x(),  halfside.y(), 0.0);
        local_points[3] = Eigen::Vector3d(-halfside.x(),  halfside.y(), 0.0);
      }
      else
        assert(0 && "Invalid shape type");
    };

    auto geom_a = geometry::FinalShape::Implementation::get_collision(*shape_a);
    auto geom_b = geometry::FinalShape::Implementation::get_collision(*shape_b);
    update_locals(geom_a, count_a, local_points_a);
    update_locals(geom_b, count_b, local_points_b);

    fcl::Transform3d tx_a, tx_b;
    motion_a->getCurrentTransform(tx_a);
    motion_b->getCurrentTransform(tx_b);

    tx_a = tx_a * shape_a->get_offset_transform();
    tx_b = tx_b * shape_b->get_offset_transform();
     
    if (initial_seperation_props.count_a == 1 && initial_seperation_props.count_b == 1)
    {
      const auto& localpoint_a = local_points_a[initial_seperation_props.pointindices_a[0]];
      const auto& localpoint_b = local_points_b[initial_seperation_props.pointindices_b[0]];
      auto point_a = tx_a * localpoint_a;
      auto point_b = tx_b * localpoint_b;

      seperation_axis = point_b - point_a;
      seperation_axis.normalize();
#ifdef DO_LOGGING
      printf("seperation_axis: %f, %f\n", seperation_axis.x(), seperation_axis.y());
#endif
    }
    else if (initial_seperation_props.count_a == 1 && initial_seperation_props.count_b == 2)
    {
      // get seperation_axis in local space
      auto& localpoint_b1 = local_points_b[initial_seperation_props.pointindices_b[0]];
      auto& localpoint_b2 = local_points_b[initial_seperation_props.pointindices_b[1]];
      
      auto vec = localpoint_b2 - localpoint_b1;
      seperation_axis = Eigen::Vector3d(vec.y(), -vec.x(), 0.0);
      seperation_axis.normalize();

      auto transformed_axis = tx_b.linear() * seperation_axis;
      local_axis_point = (localpoint_b1 + localpoint_b2) * 0.5;

      auto& localpoint_a = local_points_a[initial_seperation_props.pointindices_a[0]];
      auto point_a = tx_a * localpoint_a;
      auto point_b = tx_b * local_axis_point;

      double s = transformed_axis.dot(point_a - point_b);
      if (s < 0.0)
        seperation_axis = -seperation_axis;

#ifdef DO_LOGGING
      printf("a1 vs b2 %d, %d\n", initial_seperation_props.pointindices_b[0], initial_seperation_props.pointindices_b[1]);
      printf("localpoint_b1: %f, %f\n", localpoint_b1.x(), localpoint_b1.y());
      printf("localpoint_b2: %f, %f\n", localpoint_b2.x(), localpoint_b2.y());

      printf("seperation_axis: %f, %f\n", seperation_axis.x(), seperation_axis.y());
      printf("point_b: %f, %f\n", point_b.x(), point_b.y());
#endif
    }
    else //if (initial_seperation_props.count_a == 2)
    {
      //2 points on A
      // get seperation_axis in local space
      auto& localpoint_a1 = local_points_a[initial_seperation_props.pointindices_a[0]];
      auto& localpoint_a2 = local_points_a[initial_seperation_props.pointindices_a[1]];
      
      auto vec = localpoint_a2 - localpoint_a1;
      seperation_axis = Eigen::Vector3d(vec.y(), -vec.x(), 0.0);
      seperation_axis.normalize();

      auto transformed_axis = tx_a.linear() * seperation_axis;
      local_axis_point = (localpoint_a1 + localpoint_a2) * 0.5;

      auto point_a = tx_a * local_axis_point;

      auto& localpoint_b = local_points_b[initial_seperation_props.pointindices_b[0]];
      auto point_b = tx_b * localpoint_b;
      double s = transformed_axis.dot(point_b - point_a);
      if (s < 0.0)
        seperation_axis = -seperation_axis;

#ifdef DO_LOGGING
      printf("seperation_axis: %f, %f\n", seperation_axis.x(), seperation_axis.y());
#endif
    }
  }

  double compute_seperation(double t,
    std::shared_ptr<fcl::MotionBased> motion_a,
    std::shared_ptr<fcl::MotionBased> motion_b,
    const geometry::ConstFinalShapePtr& shape_a,
    const geometry::ConstFinalShapePtr& shape_b)
  {
    motion_a->integrate(t);
    motion_b->integrate(t);

    fcl::Transform3d tx_a, tx_b;
    motion_a->getCurrentTransform(tx_a);
    motion_b->getCurrentTransform(tx_b);

    tx_a = tx_a * shape_a->get_offset_transform();
    tx_b = tx_b * shape_b->get_offset_transform();

    auto get_support = [](const Eigen::Vector3d& axis,
      int count, Eigen::Vector3d (&local_points)[4])
    {
      int best_idx = 0;
      double best_dotp = axis.dot(local_points[0]);
      for (int i=1; i<count; ++i)
      {
        double dotp = axis.dot(local_points[i]);
        if (dotp > best_dotp)
        {
          best_idx = i;
          best_dotp = dotp;
        }
      }
      return best_idx;
    };

    // get support points on both a and b
    if (initial_seperation_props.count_a == 1 && initial_seperation_props.count_b == 1) 
    {
      // shortcut for sphere-sphere
      auto transformed_axis_a = tx_a.linear().inverse() * seperation_axis;
      auto transformed_axis_b = tx_b.linear().inverse() * -seperation_axis;
      
      support_vertex_a = get_support(transformed_axis_a, count_a, local_points_a);
      support_vertex_b = get_support(transformed_axis_b, count_b, local_points_b);

#ifdef DO_LOGGING
      printf("point-point\n");
      printf("transformed_axis_b: %f %f\n", transformed_axis_b.x(), transformed_axis_b.y());
      printf("support_vertex_a: %d support_vertex_b: %d\n", support_vertex_a, support_vertex_b);
#endif
      auto point_a = tx_a * local_points_a[support_vertex_a];
      auto point_b = tx_b * local_points_b[support_vertex_b];
      double s = seperation_axis.dot(point_b - point_a);
      return s;
    }
    else if (initial_seperation_props.count_a == 1 && initial_seperation_props.count_b == 2)
    {
      auto transformed_axis = tx_b.linear() * seperation_axis;
      auto axis_a = tx_b.linear().inverse() * -transformed_axis;

      support_vertex_a = get_support(axis_a, count_a, local_points_a);
      support_vertex_b = -1;

      auto point_a = tx_a * local_points_a[support_vertex_a];
      auto point_b = tx_b * local_axis_point;
#ifdef DO_LOGGING
      printf("  initial_seperation_props.count_b == 2, t:%f\n", t);
      printf("  transformed_axis: %f, %f\n", transformed_axis.x(), transformed_axis.y());
      printf("  axis_a: %f, %f\n", axis_a.x(), axis_a.y());
      printf("  support_vertex_a: %d\n", support_vertex_a);
      printf("  point_a: %f %f\n", point_a.x(), point_a.y());
      printf("  point_b: %f %f\n", point_b.x(), point_b.y());
#endif
      double s = transformed_axis.dot(point_a - point_b);
      return s;
    }
    else //if (initial_seperation_props.count_a == 2)
    {
      auto transformed_axis = tx_a.linear() * seperation_axis;
      auto axis_b = tx_b.linear().inverse() * -transformed_axis;
      
      support_vertex_a = -1;
      support_vertex_b = get_support(axis_b, count_b, local_points_b);

      auto point_a = tx_a * local_axis_point;
      auto point_b = tx_b * local_points_b[support_vertex_b];
#ifdef DO_LOGGING
      printf("  initial_seperation_props.count_a == 2, t:%f\n", t);
      printf("  transformed_axis: %f, %f\n", transformed_axis.x(), transformed_axis.y());
      printf("  axis_b: %f, %f\n", axis_b.x(), axis_b.y());
      printf("  point_a: %f %f\n", point_a.x(), point_a.y());
      printf("  point_b: %f %f\n", point_b.x(), point_b.y());
#endif
      double s = transformed_axis.dot(point_b - point_a);
      return s;
      
    }
  }

  double evaluate(double t,
    std::shared_ptr<fcl::MotionBased> motion_a,
    std::shared_ptr<fcl::MotionBased> motion_b,
    const geometry::ConstFinalShapePtr& shape_a,
    const geometry::ConstFinalShapePtr& shape_b)
  {
    motion_a->integrate(t);
    motion_b->integrate(t);

    fcl::Transform3d tx_a, tx_b;
    motion_a->getCurrentTransform(tx_a);
    motion_b->getCurrentTransform(tx_b);
    
    tx_a = tx_a * shape_a->get_offset_transform();
    tx_b = tx_b * shape_b->get_offset_transform();
    
    if (initial_seperation_props.count_a == 1 && initial_seperation_props.count_b == 1) 
    {
      auto point_a = tx_a * local_points_a[support_vertex_a];
      auto point_b = tx_b * local_points_b[support_vertex_b];
      double s = seperation_axis.dot(point_b - point_a);
      return s;
    }
    else if (initial_seperation_props.count_a == 1 && initial_seperation_props.count_b == 2)
    {
      auto normal = tx_b.linear() * seperation_axis;
      auto pt_a = tx_a * local_points_a[support_vertex_a];
      auto pt_b = tx_b * local_axis_point;
      double s = normal.dot(pt_a - pt_b);
      return s;
    }
    else //if (initial_seperation_props.count_a == 2)
    {
      auto normal = tx_a.linear() * seperation_axis;
      auto pt_a = tx_a * local_axis_point;
      auto pt_b = tx_b * local_points_b[support_vertex_b];
      double s = normal.dot(pt_b - pt_a);
      return s;
    }
  }
};

//==============================================================================
enum MOTION_ADVANCEMENT_RESULT
{
  ADV_RESTART = 0,
  ADV_COLLIDE,
  ADV_SEPERATED,
};

//==============================================================================
static inline MOTION_ADVANCEMENT_RESULT max_motion_advancement(double current_t,
  double t_max,
  std::shared_ptr<fcl::MotionBased> motion_a, 
  std::shared_ptr<fcl::MotionBased> motion_b,
  const geometry::ConstFinalShapePtr& shape_a,
  const geometry::ConstFinalShapePtr& shape_b,
  SeperationInfo& seperation_info,
  double target_length, double tolerance, 
  uint& iterations, double& t_out)
{
  assert(tolerance >= 0.0);
  
  target_length = target_length;
  t_out = t_max;

#ifdef DO_LOGGING
  printf("======\n");
  printf("target_length: %f\n", target_length);
#endif
  SeperationComputation computation(motion_a, motion_b, shape_a, shape_b, seperation_info);
  uint outerloop_iter = 0;
  double t1 = current_t, t2 = t_max;
  for (;;)
  {
    // find min seperation of points
    float s2 = computation.compute_seperation(t2,
      motion_a, motion_b, shape_a, shape_b);
#ifdef DO_LOGGING
    printf("outerloop_iter %d\n", outerloop_iter);
    printf("  (t2:%f, s2:%f)\n", t2, s2);
#endif
    // final configuration reached
    if (s2 > target_length + tolerance)
    {
#ifdef DO_LOGGING
      //output->state = b2TOIOutput::e_separated;
      printf("e_separated\n");
#endif
      t_out = t_max;
      return ADV_SEPERATED;
    }

    if (s2 > target_length - tolerance)
    {
      t_out = t2;
#ifdef DO_LOGGING
      printf("restart %f\n", t_out);
#endif
      return ADV_RESTART; // restart from a new configuration
    }
    
    // Compute the initial separation of the witness points.
    double s1 = computation.evaluate(t1,
      motion_a, motion_b, shape_a, shape_b);
#ifdef DO_LOGGING
    printf("  (t1:%f, s1:%f)\n", t1, s1);
#endif

    // initial overlap test
    if (s1 < target_length - tolerance)
    {
#ifdef DO_LOGGING
      //output->state = b2TOIOutput::e_failed;
      printf("e_failed, target_length:%f\n", target_length);
#endif
      t_out = t1;
      return ADV_SEPERATED;
    }

    // Check for touching
    if (s1 <= target_length + tolerance)
    {
      // Victory! t1 should hold the TOI (could be 0.0).
#ifdef DO_LOGGING
      //output->state = b2TOIOutput::e_touching;
      printf("e_touching, target_length: %f\n", target_length);
#endif
      t_out = t1;
      return ADV_COLLIDE;
    }

    // Compute 1D root of: f(x) - target_length = 0
    uint rootfind_iter = 0;
    double a1 = t1, a2 = t2;
    for (;;)
    {
      double t;
      if (rootfind_iter & 1) // Secant rule
        t = a1 + (target_length - s1) * (a2 - a1) / (s2 - s1);
      else
        t = 0.5 * (a1 + a2); // Bisection
      
      double s = computation.evaluate(t,
        motion_a, motion_b, shape_a, shape_b);
#ifdef DO_LOGGING
			printf("    (t:%f, s:%f)\n", t, s);
#endif

      if (abs(s - target_length) < tolerance)
      {
        t2 = t;
        break;
      }

      // Ensure we continue to bracket the root.
      if (s > target_length)
      {
        a1 = t;
        s1 = s;
      }
      else
      {
        a2 = t;
        s2 = s;
      }

      const uint max_rootfind_iters = 25;
      if (rootfind_iter >= max_rootfind_iters)
        break;
      ++rootfind_iter;
    }

    iterations = iterations + rootfind_iter + outerloop_iter;
    
    const uint max_otherloop = 6;
    if (outerloop_iter >= max_otherloop)
      break;
    ++outerloop_iter;
  }
#ifdef DO_LOGGING
  printf("restart\n");
#endif
  return ADV_RESTART;
}

//==============================================================================
inline bool collide_pairwise_shapes(
  std::shared_ptr<fcl::MotionBased> motion_a, 
  std::shared_ptr<fcl::MotionBased> motion_b,
  const geometry::ConstFinalShapePtr& shape_a,
  const geometry::ConstFinalShapePtr& shape_b,
  double& impact_time, uint& dist_checks, 
  uint safety_maximum_checks, double tolerance,
  double t_min = 0.0,
  double t_max = 1.0)
{
  auto calc_min_dist = [](
    const fcl::Transform3d& tx_a,
    const fcl::Transform3d& tx_b,
    const geometry::ConstFinalShapePtr& shape_a,
    const geometry::ConstFinalShapePtr& shape_b,
    SeperationInfo& seperation_info,
    double& dist,
    double& target_length)
  {
    seperation_info.count_a = 0;
    seperation_info.count_b = 0;

    auto geom_a = geometry::FinalShape::Implementation::get_collision(*shape_a);
    auto geom_b = geometry::FinalShape::Implementation::get_collision(*shape_b);

    if (geom_a->getNodeType() == fcl::GEOM_SPHERE && 
        geom_b->getNodeType() == fcl::GEOM_SPHERE)
    {
      auto sphere_a =
        std::dynamic_pointer_cast<fcl::Sphered>(geom_a);
      auto sphere_b =
        std::dynamic_pointer_cast<fcl::Sphered>(geom_b);

      auto a = tx_a.translation();
      auto b = tx_b.translation();

      target_length = sphere_a->radius + sphere_b->radius;
      
      double distsq = (b - a).squaredNorm();
      //distsq = distsq - target_length * target_length;
      dist = std::sqrt(distsq) - target_length;
      
      seperation_info.count_a = 1;
      seperation_info.pointindices_a[0] = 0;
      seperation_info.count_b = 1;
      seperation_info.pointindices_b[0] = 0;
    }
    else if (geom_a->getNodeType() == fcl::GEOM_SPHERE && 
        geom_b->getNodeType() == fcl::GEOM_BOX)
    {
      auto sphere_a =
        std::dynamic_pointer_cast<fcl::Sphered>(geom_a);
      auto box_b =
        std::dynamic_pointer_cast<fcl::Boxd>(geom_b);

      dist = 
        sphere_box_closest_features(*sphere_a, tx_a, *box_b, tx_b,
          seperation_info);
      
      target_length = sphere_a->radius;
    }
    else if (geom_a->getNodeType() == fcl::GEOM_BOX && 
        geom_b->getNodeType() == fcl::GEOM_SPHERE)
    {
      auto box_a =
        std::dynamic_pointer_cast<fcl::Boxd>(geom_a);
      auto sphere_b =
        std::dynamic_pointer_cast<fcl::Sphered>(geom_b);

      dist = 
        sphere_box_closest_features(*sphere_b, tx_b, *box_a, tx_a,
          seperation_info);
      
      //swap
      seperation_info.swap();
      target_length = sphere_b->radius;
    }
    else if (geom_a->getNodeType() == fcl::GEOM_BOX && 
        geom_b->getNodeType() == fcl::GEOM_BOX)
    {
      auto box_a =
        std::dynamic_pointer_cast<fcl::Boxd>(geom_a);
      auto box_b =
        std::dynamic_pointer_cast<fcl::Boxd>(geom_b);

      Eigen::Vector3d a(0, 0, 0), b(0, 0, 0);
      dist = 
        box_box_closest_features(*box_a, tx_a, *box_b, tx_b,
          seperation_info);
      
      target_length = 0.0;
    }
    else
      assert(0 && "Unsupported shape combination type!");
  };

  fcl::Transform3d tx_a, tx_b;
  SeperationInfo seperation_info;

  motion_a->integrate(t_min);
  motion_b->integrate(t_min);
  motion_a->getCurrentTransform(tx_a);
  motion_b->getCurrentTransform(tx_b);

  tx_a = tx_a * shape_a->get_offset_transform();
  tx_b = tx_b * shape_b->get_offset_transform();

  double target_length = 0.0;
  double dist_to_cover = 0.0;
  calc_min_dist(tx_a, tx_b, shape_a, shape_b,
    seperation_info,
    dist_to_cover, target_length);
  
  double t = t_min;
  uint iter = 0;
  while (dist_to_cover > 0.0 && t < t_max)
  {
#ifdef DO_LOGGING
    printf("======= iter:%d\n", iter);
    auto d = seperation_info.a_to_b;
    std::cout << "a_to_b: \n" << d << std::endl;
    std::cout << "dist_to_cover: " << dist_to_cover << std::endl;
    rmf_planner_viz::draw::IMDraw::draw_arrow(sf::Vector2f(0, 0), sf::Vector2f(d.x(), d.y()));
#endif
    auto collide_result = max_motion_advancement(t, t_max, motion_a, motion_b, shape_a, shape_b, 
      seperation_info,
      target_length, tolerance, dist_checks, t);
    if (collide_result == ADV_COLLIDE)
      break;
    else if (collide_result == ADV_SEPERATED)
      return false;

#ifdef DO_LOGGING
    printf("max_motion_advancement returns t: %f\n", t);
#endif

    motion_a->getCurrentTransform(tx_a);
    motion_b->getCurrentTransform(tx_b);

    tx_a = tx_a * shape_a->get_offset_transform();
    tx_b = tx_b * shape_b->get_offset_transform();

    calc_min_dist(tx_a, tx_b, shape_a, shape_b, seperation_info,
      dist_to_cover, target_length);

    ++dist_checks;
    ++iter;
    
    //infinite loop prevention. increase safety_maximum_checks if you still want a solution
    if (dist_checks > safety_maximum_checks)
      break;
  }
  
  if (dist_checks > safety_maximum_checks)
    return false;

  if (t >= t_min && t < t_max)
  {
    impact_time = t;
#ifdef DO_LOGGING
    printf("time of impact: %f\n", t);
#endif
    return true;
  }
#ifdef DO_LOGGING
  printf("no collide\n");
#endif
  return false;
}

//==============================================================================
bool collide_seperable_shapes(
  std::shared_ptr<fcl::MotionBased> motion_a, 
  std::shared_ptr<fcl::MotionBased> motion_b,
  uint sweeps,
  const geometry::ConstShapeGroup& shapes_a,
  const geometry::ConstShapeGroup& shapes_b,
  double& impact_time, uint& iterations, uint safety_maximum_iterations,
  double tolerance)
{
  for (const auto& shape_a : shapes_a)
  {
    for (const auto& shape_b : shapes_b)
    {
      uint iterations_this_pair = 0;
      double t_per_sweep = 1.0 / (double)sweeps;
      for (uint i=0; i<sweeps; ++i)
      {
        double t_min = t_per_sweep * (double)i;
        double t_max = t_per_sweep * (double)(i + 1);

        double toi = 0.0;
        bool collide = collide_pairwise_shapes(
          motion_a, motion_b, shape_a, shape_b, toi, 
          iterations_this_pair, safety_maximum_iterations,
          tolerance, t_min, t_max);

        iterations += iterations_this_pair;
        if (collide)
        {
          //we can have some leeway on accurancy of the TOI
          impact_time = toi; 
          return true;
        }

      }      
    }
  }
  return false;
}

//==============================================================================
uint get_sweep_divisions(std::shared_ptr<fcl::MotionBased> motion_a,
  std::shared_ptr<fcl::MotionBased> motion_b)
{
  motion_a->integrate(0.0);
  motion_b->integrate(0.0);

  fcl::Transform3d tx_a, tx_b;
  motion_a->getCurrentTransform(tx_a);
  motion_b->getCurrentTransform(tx_b);

  auto get_rotation_about_z = [](std::shared_ptr<fcl::MotionBased> motion, double t)
    -> double
  {
    motion->integrate(t);

    fcl::Transform3d tx;
    motion->getCurrentTransform(tx);
    
    auto mtx = tx.linear();
    Eigen::AngleAxisd angleaxis(mtx);
    
    if (angleaxis.axis().isApprox(Eigen::Vector3d(0,0,1)))
    {
      return angleaxis.angle();
    }
    else
      return 0.0;
  };

  
  double start_rot_a = get_rotation_about_z(motion_a, 0.0);
  double end_rot_a   = get_rotation_about_z(motion_a, 1.0);
  double rot_diff_a  = end_rot_a - start_rot_a;

  double start_rot_b = get_rotation_about_z(motion_b, 0.0);
  double end_rot_b   = get_rotation_about_z(motion_b, 1.0);
  double rot_diff_b  = end_rot_b - start_rot_b;

  double half_pi = EIGEN_PI * 0.5;
  uint a_intervals = (uint)std::ceil(rot_diff_a / half_pi);
  uint b_intervals = (uint)std::ceil(rot_diff_b / half_pi);
  if (a_intervals == 0 && b_intervals == 0)
    return 1;
  return a_intervals > b_intervals ? a_intervals : b_intervals;
}

} // namespace rmf_traffic