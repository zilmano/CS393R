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
#include "cvshow.h"
#include "slam.h"

#include "vector_map/vector_map.h"
#include "shared/global_utils.h"
#include <limits>

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
    rasterizer_{256, 256},
    voxels_{params_.linspace_cube, {params_.linspace_cube, params_.linspace_cube}},
    motion_voxel_{params_.linspace_cube, {params_.linspace_cube, params_.linspace_cube}},
    observ_voxel_{params_.linspace_cube, {params_.linspace_cube, params_.linspace_cube}}{}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  if (!poses_.empty()) {
    *loc = poses_.back().loc;
    *angle = poses_.back().angle;
  } else {
    *loc = Vector2f(0, 0);
    *angle = 0;
  }
}

void SLAM::ObserveLaser(const sensor_msgs::LaserScan& msg) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  static vector<Vector2f> curr_scan;
  static vector<Vector2f> curr_scan_map_frame;

  if (prev_scan_.empty()) {
      tf::proj_lidar_2_pts(msg, curr_scan,
                           CarDims::kLaserLoc,
                           params_.radar_downsample_rate,
                           true,
                           params_.lidar_range_cutoff);
      prev_scan_ = curr_scan;
      return;
  }
  if (!time_to_update_)
      return;

  cout << " ---- observe laser, time to update!" << endl;
  delta_T_.pprint("delta_T:");

  time_to_update_ = false;
  tf::proj_lidar_2_pts(msg, curr_scan,
                       CarDims::kLaserLoc,
                       params_.radar_downsample_rate,
                       true,
                       params_.lidar_range_cutoff);
  cout << "curr_scan_size:" << curr_scan.size() << endl;
  PoseSE2 predicted_pose = ExecCSM(curr_scan);
  predicted_pose.pprint("ExecCSM: predicted pose:");
  cout << "transform to global frame and add to map " << endl;
  tf::transform_points_to_glob_frame(predicted_pose, curr_scan, curr_scan_map_frame);
  cout << "Transformed scan size:" << curr_scan_map_frame.size() << endl;
  map_.insert(std::end(map_),std::begin(curr_scan_map_frame),std::end(curr_scan_map_frame));
  cout << "new map size:" << map_.size()<<endl;

  poses_.push_back(predicted_pose);
  scans_.push_back(curr_scan);
  odoms_.push_back(PoseSE2(curr_odom_loc_, curr_odom_angle_));
  prev_scan_ = curr_scan;
  prev_odom_angle_ = curr_odom_angle_;
  prev_odom_loc_ = curr_odom_loc_;

  cout << "Observe laser done." << endl;

}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
   if (!odom_initialized_) {
     prev_odom_angle_ = odom_angle;
     prev_odom_loc_ = odom_loc;
     odom_initialized_ = true;
     // Oleg: There is going to be a small error here due to odomentry noise.
     //       Unless the odomery noise is very high, it is negligelbe sinse the
     //       car only had 20ms to move.
     poses_.push_back(PoseSE2(odom_loc, odom_angle));
     poses_.back().pprint("--> Init Pose: ");
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
       curr_odom_loc_ = odom_loc;
       curr_odom_angle_ = odom_angle;
       cout << "curr odom: (" << prev_odom_loc_.x() << ","
            << prev_odom_loc_.y() << ")  a:" << odom_angle << endl;
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

    auto img = rasterizer_.rasterize(
            prev_scan_,
            params_.sigma_rasterizer);

    //normalized_imshow(Eigen::exp(img));
    //cv::waitKey(0);

    float x_start,x_end,
          y_start,y_end,
          theta_start,theta_end;

    curr_pose_mean.pprint("   odom est pose:");
    float scale_factor = 4;
     CalcCSMCube(scale_factor,
                curr_pose_mean,
                x_start, y_start, theta_start,
                x_end, y_end, theta_end);
    cout << "   CUBE dims start x:" << x_start << " end x:" << x_end <<
            " start y:" << y_start << " end y:" << y_end <<
            " ang x:" << theta_start << " end ang:" << theta_end << endl;
    float y_step = (y_end-y_start)/params_.linspace_cube;
    float x_step = (x_end-x_start)/params_.linspace_cube;
    float theta_step = fabs((theta_end-theta_start))/params_.linspace_cube;

    float x_t = x_start;
    PoseSE2 mle_pose = curr_pose_mean;
    float max_posterior_prob = std::numeric_limits<float>::lowest();

    cout << "   start going over voxels...x_step y_step theta_step" << x_step
                << y_step << theta_step <<  endl;

    std::vector<Vector3f> candidate_trans;
    std::vector<PoseSE2> candidate_poses;
    std::vector<float> candidate_motion_likelihood;
    std::vector<float> candidate_observ_likelihood;
    candidate_trans.reserve(params_.linspace_cube * params_.linspace_cube * params_.linspace_cube);
    candidate_poses.reserve(params_.linspace_cube * params_.linspace_cube * params_.linspace_cube);

    //Propose candidates
    x_t = x_start;
    for (uint i = 0; i < params_.linspace_cube; ++i) {
        float y_t = y_start;
        for (uint j = 0; j < params_.linspace_cube; ++j) {
            float theta_t = theta_start;
            for (uint k = 0; k < params_.linspace_cube; ++k) {
                Vector2f proposed_loc(x_t,y_t);
                Vector2f trans_t = Eigen::Rotation2Df(-poses_.back().angle) * (proposed_loc - poses_.back().loc);
                candidate_trans.emplace_back(trans_t(0), trans_t(1), theta_t - poses_.back().angle);
                candidate_poses.emplace_back(proposed_loc, theta_t);
                theta_t += theta_step;
            }
            y_t += y_step;
        }
        x_t += x_step;
    }

    //Motion model
    float sigma_x_y = params_.k_1*delta_T_.loc.norm()
                      + params_.k_2*fabs(delta_T_.angle);
    float sigma_theta = params_.k_3*delta_T_.loc.norm()
                        + params_.k_4*fabs(delta_T_.angle);
    cout << "sigma x_y" << sigma_x_y << endl;
    cout << "sigma theta" << sigma_theta << endl;
    Eigen::Vector3f sigmas(sigma_x_y, sigma_x_y, sigma_theta);
    auto mat = sigmas.asDiagonal();

    std::vector<Vector3f> proposed_p_list;
    proposed_p_list.reserve(candidate_poses.size());
    for (auto & p : candidate_poses)
        proposed_p_list.emplace_back(p.loc[0], p.loc[1], p.angle);
    candidate_motion_likelihood = loglikelihood_3d_mvn(proposed_p_list,
            {curr_pose_mean.loc[0], curr_pose_mean.loc[1], curr_pose_mean.angle}, mat);

    //Observation model
    //uint idx = 0;
    for (auto & bt : candidate_trans){
        Eigen::Rotation2Df rot(bt(2));
        Vector2f trans(bt(0), bt(1));
        transposed_curr_scan.clear();
        for (auto & pt : curr_scan) transposed_curr_scan.emplace_back(rot * pt + trans);
        auto obsrv_ll = rasterizer_.query(transposed_curr_scan);
        //normalized_imwrite(rasterizer_.get_qry_history(), cv::format("query_img/%d.png", idx++));
        rasterizer_.get_qry_history(true);
        float inv_lambda = params_.observ_lambda_frac / obsrv_ll.size();
        inv_lambda = (inv_lambda < 0.5)?inv_lambda:0.5;
        candidate_observ_likelihood.emplace_back(
                inv_lambda * std::accumulate(obsrv_ll.begin(), obsrv_ll.end(), 0.0f));
    }

    //Build voxels and mle
    auto motion_iter = candidate_motion_likelihood.begin();
    auto observ_iter = candidate_observ_likelihood.begin();

    x_t = x_start;
    for (uint i = 0; i < params_.linspace_cube; ++i) {
        float y_t = y_start;
        for (uint j = 0; j < params_.linspace_cube; ++j) {
            float theta_t = theta_start;
            for (uint k = 0; k < params_.linspace_cube; ++k) {
                float loglikelihood = (*motion_iter) + (*observ_iter);
                voxels_[i](j, k) = loglikelihood;
                motion_voxel_[i](j, k) = *motion_iter;
                observ_voxel_[i](j, k) = *observ_iter;

                if (loglikelihood > max_posterior_prob) {
                    max_posterior_prob = loglikelihood;
                    mle_pose = PoseSE2{x_t, y_t, theta_t};
                }

                motion_iter++;
                observ_iter++;
                theta_t += theta_step;
            }
            y_t += y_step;
        }
        x_t += x_step;
    }

    /*
    normalized_imshow(Eigen::exp(img));
    normalized_voxelshow(motion_voxel_, "motion_voxel");
    normalized_voxelshow(observ_voxel_, "observ_voxel");
    normalized_voxelshow(voxels_, "voxels");
    cv::waitKey(0);
     */
    return mle_pose;
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

PairEst SLAM::PairwiseEstimator(int index1, int index2){
  PairEst estimate;
  
  PoseSE2 odom1 = odoms_[index1];
  PoseSE2 odom2 = odoms_[index2];

  std::vector<Eigen::Vector2f> scan1 = scans_[index1];
  std::vector<Eigen::Vector2f> scan2 = scans_[index2];

  PoseSE2 est_rough;
  est_rough.loc = odom2.loc - odom1.loc;
  est_rough.angle = odom2.angle - odom1.angle;

  estimate = CSMV2(scan1, scan2, est_rough);

  return estimate;
}

PairEst SLAM::CSMV2(std::vector<Eigen::Vector2f> scan1, std::vector<Eigen::Vector2f> scan2, PoseSE2 est_rough){
  PairEst x;
  return x;
}

}  // namespace slam
