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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "xtensor/xarray.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor/xbuffer_adaptor.hpp"
#include "xtensor/xio.hpp"
#include <vector>
#include <iostream>
#include "clock.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    odom_angle_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    plot_publisher_{n},
    is_initloc_inited_{false},
    world_(SamplingConsts::downsample_rate_space, SamplingConsts::downsample_rate_time),
    latency_tracker_{plot_publisher_, 0.05f},
    latency_size_{0} {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  Clock::now();
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {

}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
    if (!is_initloc_inited_) {
        init_loc_ = robot_loc_;
        // Oleg Question: what is wrong with loc zero, not sure I understand how this works.
        auto is_loc_finite = init_loc_.allFinite();
        auto is_loc_nonzero = !init_loc_.isZero();
        if (is_loc_finite and is_loc_nonzero) {
            is_initloc_inited_ = true;
            printf("Init loc %f, %f\n", init_loc_[0], init_loc_[1]);
        }
    }
    robot_loc_ = loc;
    robot_angle_ = angle;
    robot_vel_ = vel;
    robot_omega_ = ang_vel;

    latency_tracker_.add_measurements(VelocityMeasurement{vel.norm(), ros::Time::now().toSec()});

    float curr_time = Clock::now();

    plot_publisher_.publish_named_point("Obsrv Dist", curr_time, (loc - init_loc_).norm());
    plot_publisher_.publish_named_point("Obsrv Spd", curr_time, vel.norm());
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
    laser_pcloud_local_frame_ = cloud;
}


float Navigation::ComputeDis2Stop(){
    float latency = latency_tracker_.estimate_latency();
    float actuation_latency = latency * PhysicsConsts::act_latency_portion;
    float observation_latency = latency - actuation_latency;
    float curr_spd = robot_vel_.norm();     // current speed of robot based on odometry reading
    float dis2stop = curr_spd * actuation_latency;      // distance that the robot travels over a given latency time period

    /* starting at the current observed speed (odometry) minus the speed lossed over the time period of observation latency, until the current speed is less than 0,
        decrease the current speed by the amount of speed that can be lossed over a time period of observation latency

        The total distance to stop is the sum of distances that the car travels at each given speed observed during the course of complete deceleration
    */
    for (curr_spd -= PhysicsConsts::max_acc * observation_latency; curr_spd >= 0.0;
    curr_spd -= PhysicsConsts::max_acc * observation_latency)
        dis2stop += curr_spd * latency;


    plot_publisher_.publish_named_point("Est Ltcy", Clock::now(), latency);

    return dis2stop;
}

vector<Vector2f> Navigation::ConvertLaserCloudToOdomFrame() {
	vector<Vector2f> odom_frame_point_cloud;
	odom_frame_point_cloud.clear();
	for (size_t i = 0; i < laser_pcloud_local_frame_.size(); ++i) {
		Vector2f odom_frame_point((laser_pcloud_local_frame_[i] + robot_loc_));
		odom_frame_point_cloud.push_back(odom_frame_point);
	}

	return odom_frame_point_cloud;
}


void Navigation::Test() {
	// function for debugging. Fill you test code here and run it from navigation_main (instead of changing Run).
}

void Navigation::Run() {
  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.

  float c_p = 0.01f;
  float epsilon = 0.005f;
  auto spd_inc = latency_tracker_.estimate_latency() * PhysicsConsts::max_acc;

  float dis2stop = ComputeDis2Stop() + epsilon;

  // target location is always a defined distance directly in front of the robot
  auto target_loc = driver.calculate_target_location(robot_loc_, robot_angle_);

  // Current Distance
  auto curr_dist = (target_loc - robot_loc_).norm();
  auto curr_spd = robot_vel_.norm();

  /**
   * How is curvature going to be integrated 
   */
  float desired_curvature = 0;

  driver.update_current_speed(dis2stop, is_initloc_inited_, curr_spd, curr_dist, spd_inc, c_p);
  
  drive_msg_.velocity = driver.get_velocity();
  drive_msg_.velocity = driver.drive_msg_check(drive_msg_.velocity);
  drive_msg_.curvature = desired_curvature;
  
  drive_pub_.publish(drive_msg_);

  double curr_time = ros::Time::now().toSec();
  //printf("Latency %.2f, latency samples %ld\n", latency_tracker).estimate_latency(), latency_tracker_.get_alllatencies().size());
  latency_tracker_.add_controls(VelocityControlCommand{drive_msg_.velocity, curr_time});
  plot_publisher_.publish_named_point("Ctrl cmd", Clock::now(), drive_msg_.velocity);
}

}  // namespace navigation
