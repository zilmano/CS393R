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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"
#include "shared/global_utils.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    time_to_update_(false){}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = Vector2f(0, 0);
  *angle = 0;
}

void SLAM::ObserveLaser(const sensor_msgs::LaserScan& msg) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  if (!time_to_update_)
      return;

  time_to_update_ = false;
  static vector<Vector2f> curr_scan;
  static vector<Vector2f> curr_scan_map_frame;
  tf::proj_lidar_2_pts(msg, curr_scan,
                       CarDims::kLaserLoc,
                       params_.radar_downsample_rate);

  PoseSE2 predicted_pose = ExecCSM(curr_scan);
  tf::transform_points_to_glob_frame(predicted_pose, curr_scan, curr_scan_map_frame);
  map_.insert(std::end(map_),std::begin(map_),std::end(map_));

  poses_.push_back(predicted_pose);
  prev_scan_ = curr_scan;

}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }

    /*if ((odom_loc - prev_odom_loc_).norm() < 0.0005) {
      car_moving_ = false;
    } else {
      car_moving_ = true;
    }*/

    float d_angle = odom_angle-prev_odom_angle_;
    Vector2f d_loc = odom_loc-prev_odom_loc_;

    if ( d_loc.norm() > params_.update_tresh_dist_ ||
            d_angle > params_.update_tresh_angle_) {
       delta_T_ = PoseSE2(d_loc, d_angle);
       time_to_update_ = true;
       prev_odom_angle_ = odom_angle;
       prev_odom_loc_ = odom_loc;
    }



  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
}

vector<Vector2f> SLAM::GetMap() {
  //vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map_;
}

PoseSE2 SLAM::ExecCSM(const vector<Vector2f>& curr_scan_point_cloud) const {
    return PoseSE2();
}



}  // namespace slam
