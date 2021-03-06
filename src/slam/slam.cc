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
#include "cvshow.h"
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
    curr_odom_loc_(0),
    curr_odom_angle_(0),
    odom_initialized_(false),
    time_to_update_(false),
    viz_pub_(NULL),
    viz_msg_(NULL),
    rasterizer_{256, 256}{}

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

  cout << " ---- observe laser, time to update!" << endl << endl;
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
       cout << "Observe_Odom - prev_odom: (" << prev_odom_loc_.x() << ","
            << prev_odom_loc_.y() << ")  a:" << prev_odom_angle_ << endl;
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
    static vector<Vector2f> backproj_scan;
    PoseSE2 prev_pose  = poses_.back();
    PoseSE2 curr_pose_mean = prev_pose + delta_T_;

    Eigen::MatrixXf lookup_img = rasterizer_.rasterize(
            prev_scan_,
            params_.sigma_rasterizer);
    normalized_imwrite(Eigen::exp(lookup_img.array()));

    curr_pose_mean.pprint("   odom estimated pose:");
    cout << "   curr odom pose: (" << curr_odom_loc_.x()
         << "," << curr_odom_loc_.y() << ") " << curr_odom_angle_ << endl;

    float x_start,x_end,
          y_start,y_end,
          theta_start,theta_end;
    float scale_factor = 4;
    CalcCSMCube(scale_factor,
                curr_pose_mean,
                x_start, y_start, theta_start,
                x_end, y_end, theta_end);
    float y_step = (y_end-y_start)/params_.linspace_cube;
    float x_step = (x_end-x_start)/params_.linspace_cube;
    float theta_step = fabs((theta_end-theta_start))/params_.linspace_cube;

    PoseSE2 mle_pose = curr_pose_mean;
    float max_posterior_prob = std::numeric_limits<float>::lowest();
    cout << "   start going over voxels...x_step y_step theta_step" << x_step
                    << y_step << theta_step <<  endl;
    float x_t = x_start;
    vector<Vector2f> mle_backproj_scan;
    for (int i = 0; i <= params_.linspace_cube; ++i) {
       float y_t = y_start;
       for (int j = 0; j <=  params_.linspace_cube; ++j) {
           float theta_t = theta_start;
           for (int k = 0; k <=  params_.linspace_cube; ++k) {
                Vector2f loc_t(x_t,y_t);

                //PoseSE2 candidate_d_T_map_frame = delta_T_;
                PoseSE2 candidate_pose(loc_t,theta_t);
                PoseSE2 d_T_prev_pose_frame = tf::transform_pose_to_loc_frame(
                        prev_pose,
                        candidate_pose);
                //PoseSE2(loc_t,theta_t).pprint("Sanity test ",true);
                tf::transform_points_to_glob_frame(d_T_prev_pose_frame,
                        curr_scan,
                        transposed_curr_scan);
                /*for (size_t i = 0; i < curr_scan.size(); i++) {
                    debug::print_loc(curr_scan[i], "        loc point", false);
                    debug::print_loc(transposed_curr_scan[i], " transposed ", true);
                }*/
                float posterior_prob  = CalcPoseMLE(transposed_curr_scan,
                    candidate_pose,
                    curr_pose_mean);

                if (posterior_prob > max_posterior_prob) {
                     max_posterior_prob = posterior_prob;
                     //mle_pose = PoseSE2(x_t,y_t,theta_t);
                     mle_pose = candidate_pose;
                     mle_backproj_scan = transposed_curr_scan;
                }
                theta_t += theta_step;
            }
            y_t += y_step;
        }
        x_t += x_step;
    }
    rasterizer_.get_qry_history(true);

    mle_pose.pprint("   MLE estimated pose: ");
    cout << "  Probs of scan of estimated pose --> ";
    CalcPoseMLE(mle_backproj_scan,
                    mle_pose,
                    curr_pose_mean,true);
    Eigen::MatrixXf scan_img1 = rasterizer_.get_qry_history();
    normalized_imwrite(scan_img1,"mle_pose_backproj.png");
    rasterizer_.get_qry_history(true);

    if (viz_pub_) {
        visualization::ClearVisualizationMsg(*viz_msg_);
        visualization::DrawPointCloud(prev_scan_, 0x0000FF, *viz_msg_);
        //visualization::DrawPointCloud(transposed_curr_scan, 0xFF00, *viz_msg_);
        visualization::DrawPointCloud(mle_backproj_scan, 0xFF0000, *viz_msg_);
     }


    PoseSE2 d_T_prev_pose_frame = tf::transform_pose_to_loc_frame(
            prev_pose,
            curr_pose_mean);
    tf::transform_points_to_glob_frame(d_T_prev_pose_frame,
            curr_scan,
            transposed_curr_scan);
    cout << "  Probs of scan of real pose -->";
    CalcPoseMLE(transposed_curr_scan,
                            curr_pose_mean,
                            curr_pose_mean,true);
    visualization::DrawPointCloud(transposed_curr_scan, 0xFF00, *viz_msg_);
    Eigen::MatrixXf scan_img2 = rasterizer_.get_qry_history();
    normalized_imwrite(scan_img2,"best_pose_backproj.png");
    if (viz_pub_)
        viz_pub_->publish(*viz_msg_);

    //cout << "   done " << endl;
    //mle_pose = curr_pose_mean;

    return mle_pose;
}

float SLAM::CalcPoseMLE(const vector<Vector2f>& transposed_scan,
                    PoseSE2 proposed_pose,
                    PoseSE2 mean_pose,bool debug) {
  // Get list of probabilities of each laser point based on rasterized image
  vector<float> obs_prob = rasterizer_.query(transposed_scan);
  float inv_lambda = params_.observ_lambda_frac / obs_prob.size();
  inv_lambda = (inv_lambda < 0.5)?inv_lambda:0.5;

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
  //cout << "sigma x_y" << sigma_x_y << endl;
  //cout << "sigma theta" << sigma_theta << endl;

  Eigen::Vector3f sigmas(sigma_x_y, sigma_x_y, sigma_theta);
  auto mat = sigmas.asDiagonal();
  
  // calculate probability of predicted pose given mean pose
  vector<float> llh = loglikelihood_3d_mvn(proposed_p_list, mean_p, mat);
  float motion_prob = std::accumulate(llh.begin(), llh.end(), 0.0f);

  if (debug) {
    cout << "            motion prior log-prob: " << motion_prob << endl;
    cout << "            obs log prob: "
         << std::accumulate(obs_prob.begin(), obs_prob.end(), 0.0f) << endl;
  }
  return inv_lambda*std::accumulate(obs_prob.begin(), obs_prob.end(), 0.0f) + motion_prob;

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
