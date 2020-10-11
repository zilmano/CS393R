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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
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
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

#include "shared/constants.h"
#include "shared/global_utils.h"

#include "ros/ros.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "visualization/visualization.h"


using geometry::line2f;
using geometry::Line;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace {
    ros::Publisher* viz_pub_ = NULL;
    amrl_msgs::VisualizationMsg* viz_msg_ = NULL;
}

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    obs_likelihood(1,PhysicsConsts::radar_noise_std),
    map_loaded_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            unsigned int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_points_ptr,
                                            vector<float>* scan_ranges_ptr) {

  //if (!map_loaded_)
  //    return;

  cout << "GetPredictedPointCloud" << endl;
  vector<Vector2f>& scan_points = *scan_points_ptr;
  vector<float>& scan_ranges = *scan_ranges_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan_points.resize(num_ranges);
  scan_ranges.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan_ranges.size(); ++i) {
    scan_points[i] = Vector2f(0, 0);
    scan_ranges[i] = 0;
  }

  // Poor man's scene render returning the segments that are in radar range.
  vector<Line<float>> lines_in_range;
  //cout << "map line num: " << map_.lines.size() << endl;
  for (size_t i = 0; i < map_.lines.size(); ++i) {

        Line<float>  segment_in_radar_range = geometry::get_sub_segment_in_circle(
                                                                map_.lines[i],
                                                                loc, range_max);
        if (segment_in_radar_range.Length() > GenConsts::kEpsilon) {
            /*cout << " orig line: (" << map_.lines[i].p0.x()<< "," << map_.lines[i].p0.y()
                 << ")-(" << map_.lines[i].p1.x()<< "," << map_.lines[i].p1.y() << ")" << endl;
            cout << " segment: (" << segment_in_radar_range.p0.x()<< "," << segment_in_radar_range.p0.y()
                 << ")-(" << segment_in_radar_range.p1.x()<< "," << segment_in_radar_range.p1.y()
                 << ")" << endl;*/

            lines_in_range.push_back(segment_in_radar_range);
        }
    }

  //std::cout << "segments in range" << lines_in_range.size() << std::endl;
  float angle_incr = (angle_max - angle_min)/num_ranges;
  float beam_angle = angle + angle_min;
  // Set the expected range for each radar angle, based on the rendered part of the map
  for (unsigned int range_index = 0; range_index < num_ranges; ++range_index) {

      //cout << "beam angle:" << beam_angle*360/(2*M_PI) << endl;
      Vector2f radar_ray_endpoint(cos(beam_angle)*range_max,sin(beam_angle)*range_max);
      radar_ray_endpoint += loc;
      Line<float> ray_segment(loc,radar_ray_endpoint);

      Vector2f closest_intersection(radar_ray_endpoint);
      //cout << "beam start range:" << ray_segment.Length() << endl;
      float dist_to_closest = range_max;


      for (size_t i = 0; i < lines_in_range.size(); ++i) {
          const Line<float> map_line = lines_in_range[i];
          Vector2f intersect_point;
          bool intersects = map_line.Intersection(ray_segment, &intersect_point);
          if (intersects) {
              /*printf("   Intersects at %f,%f\n",
                     intersect_point.x(),
                     intersect_point.y());
              cout << "   line: (" << map_line.p0.x()<< "," << map_line.p0.y()
                   << ")-(" << map_line.p1.x()<< "," << map_line.p1.y() << endl;*/
              float dist_to_intersect_point = (intersect_point - loc).norm();
              //cout << "   new range:" << dist_to_intersect_point << endl;
              if (dist_to_intersect_point < dist_to_closest) {
                    dist_to_closest = dist_to_intersect_point;
                    closest_intersection = intersect_point;
              }
          }
      }
      scan_points[range_index] = closest_intersection;
      scan_ranges[dist_to_closest];
      beam_angle += angle_incr;
  }

}

void ParticleFilter::Update(const vector<float>& ranges,
                            unsigned int num_ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.

  static vector<Vector2f> scan;
  static vector<float> expected_ranges;
  //cout << "update::" << endl;
  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle,num_ranges, range_min,
                         range_max, angle_min, angle_max, &scan, &expected_ranges);

  //OLEG: Sanity check. Remove later.
  if (scan.size() !=  expected_ranges.size())
      throw "Internal error. ParticleFilter::Update: scan size not equal expected_ranges size.";


  /*for (const auto &p : scan) {
        cout << p.x()<< "," << p.y() << " ";
  }*/
  //cout << endl << "scan size:" << scan.size() << " " << expected_ranges.size() << endl;
  if (viz_pub_) {
      //cout << "print cloud" << endl;
      visualization::ClearVisualizationMsg(*viz_msg_);
      visualization::DrawPointCloud(scan, 0xFF00, *viz_msg_);
      viz_pub_->publish(*viz_msg_);
  }
  /*float p_weight = obs_likelihood.calculate_accumulated_loglikelihood(
      expected_ranges,
      ranges);*/
  p_ptr->weight = 1;
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  vector<Particle> new_particles;
  vector<Eigen::Vector2f> number_line;
  number_line.resize(weights_.size());
  number_line[0] = Vector2f(0,weights_[0]);

  for (size_t i = 1; i < number_line.size(); i++){
    float lower = number_line[i-1].y();
    float upper = number_line[i-1].y() + weights_[i];
    number_line[i] = Vector2f(lower,upper);
  }

  for(size_t i = 0; i < particles_.size(); i++){
    float x = rng_.UniformRandom(0,1);
    for(size_t j = 0; j < number_line.size(); j++){
      if((x > number_line[i].x()) and (x <= number_line[i].y())){
        new_particles.push_back(particles_[j]);
        break;
      }
    }
  }

  particles_ = new_particles;

  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  //float x = rng_.UniformRandom(0, 1);
  //printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
        
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max,
                                  float angle_incr) {

  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  unsigned int n = pf_params_.radar_downsample_rate;
  unsigned int num_ranges;
  float angle_max_true;
  if (n != 1) {
      unsigned int downsample_truncated_pts = ((ranges.size()-1)%n);
      //cout << "downsample_truncated_pts:" << downsample_truncated_pts;
      num_ranges = std::floor((ranges.size()-downsample_truncated_pts)/n)+1;
      angle_max_true = angle_min + (ranges.size()-downsample_truncated_pts-1)*angle_incr;
  } else {
      num_ranges = ranges.size();
      angle_max_true = angle_min + (ranges.size()-1)*angle_incr;
  }
  static vector<float> downsampled_ranges;
  downsampled_ranges.resize(num_ranges);
  //cout << "LIDAR scan size:" << ranges.size() << endl;
  unsigned int downsample_cnt = 0;
  for (size_t i=0; i < ranges.size(); ++i) {
        if (i%n == 0) {
            downsampled_ranges[downsample_cnt] = ranges[i];
            downsample_cnt++;
        }
  }
  //cout << "test cnt:" << downsample_cnt << "downsampled_sizes:" << downsampled_ranges.size() << endl;
  // OLEG TODO: sanity check remove later
  if (downsample_cnt != num_ranges) {
      throw "Internal Error.ParticleFilter::ObserveLaser: Error with num_ranges calculation found";
  }
  for (auto &p: particles_) {
        Update(downsampled_ranges,
               num_ranges,
               range_min,
               range_max,
               angle_min,
               angle_max_true,
               &p);
  }
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.


  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  float x = rng_.Gaussian(0.0, 2.0);
  printf("Random number drawn from Gaussian distribution with 0 mean and "
         "standard deviation of 2 : %f\n", x);
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
    cout << "loading map...." << endl;
    map_.Load(map_file);
    particles_.push_back(Particle(loc, angle, 0));
    map_loaded_ = true;

}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;
}

void ParticleFilter::SetRosHandleAndInitPubs(ros::Publisher* pub,
                                             amrl_msgs::VisualizationMsg* msg) {
    viz_msg_ = msg;
    viz_pub_ = pub;
}


}  // namespace particle_filter
