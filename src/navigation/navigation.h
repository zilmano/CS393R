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
#include "constants.h"
#include "plotpublisher.h"
#include "world.h"
#include "drivingcontrols.h"
#include "collisionplanner.h"
#include "state_estimator.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {


struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

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

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  //Added public methods go in this section
  float RePlanPath();
  float SetOptimalVelocity(float target_dist=0);
  void Test();
  
  // Added private methods go in this section
 private:

   float ComputeDis2Stop();
   std::vector<Eigen::Vector2f> ConvertLaserCloudToOdomFrame();


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
  PlotPublisher plot_publisher_;
  
  float robot_curvature_;
  Eigen::Vector2f robot_center_of_turning_;
  Eigen::Vector2f robot_target_loc_;

  Eigen::Vector2f init_loc_;
  float init_angle_;
  bool is_initloc_inited_;
  
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
  
};

}  // namespace navigation

#endif  // NAVIGATION_H
