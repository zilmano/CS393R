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
#include <cmath>

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

using navigation::PoseSE2;

namespace slam {

struct SlamParams {
    explicit SlamParams(): radar_downsample_rate(1),
                            k_1(1), k_2(1), k_3(0.3), k_4(1),
                            update_tresh_angle_(M_PI/3),
                            update_tresh_dist_(0.5){}
    unsigned int radar_downsample_rate;
    //unsigned int resample_n_step;
    //float particle_init_sigma;
    //float d_long;
    //float d_short;
    float k_1;
    float k_2;
    float k_3;
    float k_4;
    float update_tresh_angle_;
    float update_tresh_dist_;
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

  PoseSE2 ExecCSM(const std::vector<Eigen::Vector2f>& curr_scan_point_cloud) const;

  float CalcPoseMLE(const Eigen::ArrayXXf& lookup_table, const std::vector<Eigen::Vector2f>& transposed_scan) {return 0;};
  float LocProbMotionModel(const Eigen::Vector2f& loc,const PoseSE2& mean, Eigen::Matrix3f cov) {return 0;};

 private:


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

  SlamParams params_;
};
}  // namespace slam

#endif   // SRC_SLAM_H_
