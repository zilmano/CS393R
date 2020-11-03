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
#include "2d_normal.h"
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
    time_to_update_(false),
    rasterizer_{256, 256}{}

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
  map_.insert(std::end(curr_scan_map_frame),std::begin(map_),std::end(map_));

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

    if ( d_loc.norm() > params_.update_tresh_dist ||
            d_angle > params_.update_tresh_angle) {
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

PoseSE2 SLAM::ExecCSM(const vector<Vector2f>& curr_scan) {
    static vector<Vector2f> transposed_curr_scan;
    PoseSE2 prev_pose  = poses_.back();
    PoseSE2 curr_pose_mean = prev_pose + delta_T_;

    Eigen::ArrayXXf lookup_table = rasterizer_.rasterize(
            prev_scan_,
            params_.sigma_rasterizer);

    float scale_factor = 1;
    float x_start,x_end,
          y_start,y_end,
          theta_start,theta_end;

    CalcCSMCube(scale_factor,
                curr_pose_mean,
                x_start, y_start, theta_start,
                x_end, y_end, theta_end);

    float y_step = (y_end-y_start)/params_.linspace_cube;
    float x_step = (x_end-x_start)/params_.linspace_cube;
    float theta_step = (theta_end-theta_start)/params_.linspace_cube;

    float x_t = x_start;
    float y_t = y_start;
    float theta_t = theta_start;
    PoseSE2 mle_pose = curr_pose_mean;
    float max_posterior_prob = 0;
    for (int i = 0; i <= params_.linspace_cube; ++i) {
        for (int j = 0; j <=  params_.linspace_cube; ++j) {
            for (int k = 0; k <=  params_.linspace_cube; ++k) {
                 Vector2f loc_t(x_t,y_t);
                 PoseSE2 candidate_d_T(loc_t-prev_pose.loc,theta_t-prev_pose.angle);
                 PoseSE2 candidate_pose = prev_pose + candidate_d_T;
                 tf::transform_points_to_loc_frame(candidate_d_T,
                                                   curr_scan,
                                                   transposed_curr_scan);

                 float posterior_prob = CalcPoseMLE(transposed_curr_scan,
                                                    candidate_pose,
                                                    curr_pose_mean);
                 if (posterior_prob > max_posterior_prob) {
                     max_posterior_prob = posterior_prob;
                     mle_pose = PoseSE2(x_t,y_t,theta_t);
                 }
                 theta_t += theta_step;
            }
            y_t += y_step;
        }
        x_t += x_step;
    }

    return mle_pose;
}

float SLAM::CalcPoseMLE(const vector<Vector2f>& transposed_scan,
                    PoseSE2 proposed_pose,
                    PoseSE2 mean_pose) {
  // Get list of probabilities of each laser point based on rasterized image
  vector<float> obs_prob = rasterizer_.query(transposed_scan);

  // proposed pose
  float prop_x = proposed_pose.loc.x();
  float prop_y = proposed_pose.loc.y();
  float prop_ang = proposed_pose.angle;
  Eigen::Vector3f proposed_p(prop_x,prop_y,prop_ang);
  vector<Eigen::Vector3f> proposed_p_list;
  proposed_p_list.push_back(proposed_p);

  // mean pose
  float mean_x = mean_pose.loc.x();
  float mean_y = mean_pose.loc.y();
  float mean_ang = mean_pose.angle;
  Eigen::Vector3f mean_p(mean_x,mean_y,mean_ang);

  // covariance
  float sigma_x_y = params_.k_1*delta_T_.loc.norm()
                         + params_.k_2*fabs(delta_T_.angle);

  float sigma_theta = params_.k_3*delta_T_.loc.norm()
                            + params_.k_4*fabs(delta_T_.angle);

  Eigen::Vector3f sigmas(sigma_x_y, sigma_x_y, sigma_theta);
  auto mat = sigmas.asDiagonal();
  
  // calculate probability of predicted pose given mean pose
  vector<float> llh = loglikelihood_3d_mvn(proposed_p_list, mean_p, mat);
  float motion_prob = std::accumulate(llh.begin(), llh.end(), 0);

  // observation model * motion model
  transform(obs_prob.begin(), obs_prob.end(), obs_prob.begin(), [motion_prob](float &c){ return c + motion_prob; });

  return std::accumulate(obs_prob.begin(), obs_prob.end(), 0); 
}

void SLAM::CalcCSMCube(float scale_factor,
                       const PoseSE2& curr_pose_mean,
                       float &start_x,
                       float &start_y,
                       float &start_theta,
                       float &end_x,
                       float &end_y,
                       float &end_theta) {
    // Start and end ranges for all 3 dimensions of the cube around the x_t pose.
    // This is the variance for each dimension*some_scaling_factor
    // Variance is determined according to lecture 8 particle filter motion model.
    if (scale_factor <= 0)
        throw "scale_factor has to be a positive number.";

    float sigma_x_y = params_.k_1*delta_T_.loc.norm()
                         + params_.k_2*fabs(delta_T_.angle);

    float sigma_theta = params_.k_3*delta_T_.loc.norm()
                            + params_.k_4*fabs(delta_T_.angle);

    start_x = curr_pose_mean.loc.x() - scale_factor*sigma_x_y;
    start_y = curr_pose_mean.loc.y() - scale_factor*sigma_x_y;
    start_theta = curr_pose_mean.angle - scale_factor*sigma_theta;
    end_x = curr_pose_mean.loc.x() + scale_factor*sigma_x_y;
    end_y = curr_pose_mean.loc.y() + scale_factor*sigma_x_y;
    end_theta = curr_pose_mean.angle + scale_factor*sigma_theta;
}

}  // namespace slam
