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
\file    particle-filter.h
\brief   Particle Filter Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/line2d.h"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"
#include "amrl_msgs/VisualizationMsg.h"
//#include "glog/logging.h"

#include "observation_model.h"

#ifndef SRC_PARTICLE_FILTER_H_
#define SRC_PARTICLE_FILTER_H_

// Forward Declarations
namespace ros {
   class Publisher;
}

namespace particle_filter {

struct Particle {
  Particle(): loc(0),angle(0),weight(0) {};
  explicit Particle(Eigen::Vector2f init_loc, float init_angle, double init_weight):
                    loc(init_loc),
                    angle(init_angle),
                    weight(init_weight) {};
  Eigen::Vector2f loc;
  float angle;
  double weight;
};

struct PfParams {
    explicit PfParams(): radar_downsample_rate(1),d_long(0),d_short(0),
                         k_1(1), k_2(1), sigma_obs(1), gamma(1), num_particles(50) {}
    unsigned int radar_downsample_rate;
    float d_long;
    float d_short;
    float k_1;
    float k_2;
    float sigma_obs;
    float gamma;
    unsigned int num_particles;
};

class ParticleFilter {
 public:
  // Default Constructor.
  ParticleFilter();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max,
                    float angle_incr);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Initialize the robot location.
  void Initialize(const std::string& map_file,
                  const Eigen::Vector2f& loc,
                  const float angle);

  // Return the list of particles.
  void GetParticles(std::vector<Particle>* particles) const;

  // Get robot's current location.
  void GetLocation(Eigen::Vector2f* loc, float* angle) const;

  // Update particle weight based on laser.
  void Update(const std::vector<float>& ranges,
              unsigned int num_ranges,
              float range_min,
              float range_max,
              float angle_min,
              float angle_max,
              Particle* p);

  // Resample particles.
  void Resample();

  // For debugging: get predicted point cloud from current location.
  void GetPredictedPointCloud(const Eigen::Vector2f& loc,
                              const float angle,
                              unsigned int num_ranges,
                              float range_min,
                              float range_max,
                              float angle_min,
                              float angle_max,
                              std::vector<Eigen::Vector2f>* scan_points,
                              std::vector<float>* scan_ranges);
  /*
   * Our added public members
   */
  void SetParams(const PfParams& params) {
      pf_params_ = params;
      obs_likelihood.setGamma(pf_params_.gamma);
      obs_likelihood.setSigma(pf_params_.sigma_obs);

  }

  void SetRosHandleAndInitPubs(ros::Publisher* pub,
                               amrl_msgs::VisualizationMsg* msg);

  bool isInited() {
      return map_loaded_;
  }

 private:

  // List of particles being tracked.
  std::vector<Particle> particles_;

  // Map of the environment.
  vector_map::VectorMap map_;

  // Random number generator.
  util_random::Random rng_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  // Our members
  ObservationModel obs_likelihood;
  PfParams pf_params_;
  bool map_loaded_;




};
}  // namespace slam

#endif   // SRC_PARTICLE_FILTER_H_
