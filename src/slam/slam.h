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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/global_utils.h"
#include "sensor_msgs/LaserScan.h"
#include "rasterizer.h"
#include <cmath>

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

using navigation::PoseSE2;

namespace slam {

struct SlamParams {
    explicit SlamParams(): radar_downsample_rate(1),
                            k_1(1), k_2(1), k_3(0.3), k_4(1),
                            update_tresh_angle(M_PI/3),
                            update_tresh_dist(0.5),
                            linspace_cube(20){
        sigma_rasterizer <<  1.75e-3, 1e-3, 1e-3, 1.75e-3;
    }
    unsigned int radar_downsample_rate;
    //unsigned int resample_n_step;
    //float particle_init_sigma;
    //float d_long;
    //float d_short;
    float k_1;
    float k_2;
    float k_3;
    float k_4;
    float update_tresh_angle;
    float update_tresh_dist;
    float linspace_cube;
    Eigen::Matrix2f sigma_rasterizer;
    //float sigma_obs;
    //float gamma;
};


class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const sensor_msgs::LaserScan& msg);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

  PoseSE2 ExecCSM(const std::vector<Eigen::Vector2f>& curr_scan_point_cloud);

  float CalcPoseMLE(const std::vector<Eigen::Vector2f>& transposed_scan,
                    PoseSE2 proposed_pose,
                    PoseSE2 mean_pose);
  //float LocProbMotionModel(const Eigen::Vector2f& loc,const PoseSE2& mean, Eigen::Matrix3f cov) {return 0;};

 private:
  void CalcCSMCube(float scale_factor,
                   const PoseSE2& curr_pose_mean,
                   float &start_x,
                   float &start_y,
                   float &start_theta,
                   float &end_x,
                   float &end_y,
                   float &end_theta);


 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  //vector<vector<Vector2f>> scans_;
  std::vector<Eigen::Vector2f> prev_scan_;
  std::vector<PoseSE2> poses_;
  std::vector<Eigen::Vector2f> map_;

  PoseSE2 delta_T_;
  bool time_to_update_;

  Rasterizer rasterizer_;
  SlamParams params_;
};
}  // namespace slam

#endif   // SRC_SLAM_H_
