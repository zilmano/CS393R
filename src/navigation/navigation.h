//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"
#include "latencytracking.hpp"
#include "shared/constants.h"
#include "plotpublisher.h"
#include "world.h"
#include "drivingcontrols.h"
#include "collisionplanner.h"
#include "state_estimator.h"
#include "global_planner.h"
#include "vector_map/vector_map.h"
#include <iostream>

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

using planning::A_star;
using planning::Graph;

namespace navigation {
struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct NavParams {
  float plan_grid_pitch;
  int plan_x_start;
  int plan_x_end;
  int plan_y_start;
  int plan_y_end;
  int plan_num_of_orient;
  float plan_margin_to_wall;
  float replan_dist;
  float pure_pursuit_circ_rad;
  float w_fpl;
  float w_clr;
  float w_dist;
  float obs_min_clearance;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, NavParams params, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continuously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  //Added public methods go in this section
  vector_map::VectorMap LoadMap(const std::string& map_file);
  float FindDistToGoal(float curvature, float arc_length,
                       Vector2f goal_loc, bool count_bounday_points=false);
  float FindMinClearance(float c, float arc_length,
                         const std::vector<Vector2f>& work_point_cloud);
  Eigen::Vector2f PlanLocalPath(Vector2f goal_loc);
  float SetOptimalVelocity(float target_dist=0, float curvature = 0);
  void Test();
  


 // Added private methods go in this section
 private:

   float ComputeDis2Stop();
   std::vector<Eigen::Vector2f> ConvertLaserCloudToOdomFrame();
   std::vector<Eigen::Vector2f> ConvertLaserCloudToOtherFrame(const Vector2f& loc,float angle);
   void visPlan();
   inline void PrintDbg(const std::string &msg) {
       if ((step_num_ % Debug::dbg_print_rate) == 0) {
           std::cout << "DBG::<STEP " << step_num_ << "> " << msg << std::endl;
       }
   };


 private:

  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

  // Added member variables start here

  std::vector<Eigen::Vector2f> laser_pcloud_local_frame_;
  PoseSE2 estimate_pose_local_frame_;

  PlotPublisher plot_publisher_;

  Eigen::Vector2f init_loc_;
  bool is_initloc_inited_;
  
  // Counter for step since start-up
  unsigned int step_num_;
  // Representation of our world.
  World world_;
    
  // Latency management
  LatencyTracking<VelocityMeasurement, VelocityControlCommand, true> latency_tracker_;
  unsigned long latency_size_;

   // speed and turning controls
  DrivingControls driver;
  
  // Processors
  StateEstimator state_estimator_;
  CollisionPlanner collision_planner_;
  
  NavParams nav_params_;
  Graph graph_;
  A_star glob_planner_;

  std::list<planning::GraphIndex> plan_;

};

};  // namespace navigation

#endif  // NAVIGATION_H
