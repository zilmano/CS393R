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
\file    particle-filter-main.cc
\brief   Main entry point for particle filter based
         mobile robot localization
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <termios.h>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "std_msgs/String.h"
#include "gflags/gflags.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/package.h"

#include "config_reader/config_reader.h"
#include "shared/math/math_util.h"
#include "shared/math/line2d.h"
#include "shared/util/timer.h"

#include "particle_filter.h"
#include "vector_map/vector_map.h"
#include "visualization/visualization.h"
#include "observation_model.h"

// OLEG DBG:: Remove.
#include <iostream>
#include <shared/global_utils.h>
using Eigen::Vector2f;
using std::endl;
using std::cout;

using amrl_msgs::VisualizationMsg;
using geometry::line2f;
using geometry::Line;
using math_util::DegToRad;
using math_util::RadToDeg;
using ros::Time;
using std::string;
using std::vector;
using Eigen::Vector2f;
using visualization::ClearVisualizationMsg;
using visualization::DrawArc;
using visualization::DrawPoint;
using visualization::DrawLine;
using visualization::DrawParticle;

// Create command line arguments
DEFINE_string(laser_topic, "/scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "/odom", "Name of ROS topic for odometry data");
DEFINE_string(init_topic,
              "/set_pose",
              "Name of ROS topic for initialization");
DEFINE_string(map, "", "Map file to use");
DEFINE_string(pf_topic, "/pf", "Particle filter results");

DECLARE_int32(v);

// Create config reader entries
CONFIG_STRING(map_name_, "map");
CONFIG_FLOAT(init_x_, "init_x");
CONFIG_FLOAT(init_y_, "init_y");
CONFIG_FLOAT(init_r_, "init_r");
config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

bool run_ = true;
particle_filter::ParticleFilter particle_filter_;
ros::Publisher visualization_publisher_;
ros::Publisher localization_publisher_;
ros::Publisher laser_publisher_;
ros::Publisher pfresult_publisher_;
VisualizationMsg vis_msg_;
sensor_msgs::LaserScan last_laser_msg_;
vector_map::VectorMap map_;
vector<Vector2f> trajectory_points_;

//OLEG TODO: remove debug variables
Vector2f dbg_init_loc_;
float dbg_init_angle_;


void InitializeMsgs() {
  std_msgs::Header header;
  header.frame_id = "map";
  header.seq = 0;

  vis_msg_ = visualization::NewVisualizationMessage("map", "particle_filter");
}

void PublishParticles() {
  vector<particle_filter::Particle> particles;
  particle_filter_.GetParticles(&particles);
  for (const particle_filter::Particle& p : particles) {
    DrawParticle(p.loc, p.angle, vis_msg_);
  }
}

void PublishPredictedScan() {
  //const uint32_t kColor = 0xd67d00;
  const uint32_t kColor = 0xFF00;
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  static vector<Vector2f> predicted_scan;
  static vector<float> predicted_ranges;
  particle_filter_.GetPredictedPointCloud(
      robot_loc,
      robot_angle,
      last_laser_msg_.ranges.size(),
      last_laser_msg_.range_min,
      last_laser_msg_.range_max,
      last_laser_msg_.angle_min,
      last_laser_msg_.angle_max,
      &predicted_scan,
      &predicted_ranges);
  for (const Vector2f& p : predicted_scan) {
    DrawPoint(p, kColor, vis_msg_);
  }

  //Publish pf results
  char msg[512];
  std_msgs::String strmsg;
  sprintf(msg, "%f %f %f", robot_loc[0], robot_loc[1], robot_angle);
  strmsg.data = std::string(msg);
  pfresult_publisher_.publish(strmsg);
}

void PublishTrajectory() {
  const uint32_t kColor = 0xadadad;
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  static Vector2f last_loc_(0, 0);
  if (!trajectory_points_.empty() &&
      (last_loc_ - robot_loc).squaredNorm() > Sq(1.5)) {
    trajectory_points_.clear();
  }
  if (trajectory_points_.empty() ||
      (robot_loc - last_loc_).squaredNorm() > 0.25) {
    trajectory_points_.push_back(robot_loc);
    last_loc_ = robot_loc;
  }
  for (size_t i = 0; i + 1 < trajectory_points_.size(); ++i) {
    DrawLine(trajectory_points_[i],
             trajectory_points_[i + 1],
             kColor,
             vis_msg_);
  }
}

void PublishVisualization() {
  static double t_last = 0;
  if (GetMonotonicTime() - t_last < 0.05) {
    // Rate-limit visualization.
    return;
  }
  cout << "Publish..." << endl;
  t_last = GetMonotonicTime();
  vis_msg_.header.stamp = ros::Time::now();
  ClearVisualizationMsg(vis_msg_);

  PublishParticles();
  PublishPredictedScan();
  PublishTrajectory();
  visualization_publisher_.publish(vis_msg_);
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f\n", msg.header.stamp.toSec());
  }
  last_laser_msg_ = msg;
  particle_filter_.ObserveLaser(
      msg.ranges,
      msg.range_min,
      msg.range_max,
      msg.angle_min,
      msg.angle_max,
      msg.angle_increment);
  PublishVisualization();
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  const Vector2f odom_loc(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float odom_angle =
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  particle_filter_.ObserveOdometry(odom_loc, odom_angle);
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  amrl_msgs::Localization2DMsg localization_msg;
  localization_msg.pose.x = robot_loc.x();
  localization_msg.pose.y = robot_loc.y();
  localization_msg.pose.theta = robot_angle;
  localization_publisher_.publish(localization_msg);
  // Visualize Real location in simulation/rosbag.
  /*vis_msg_.header.stamp = ros::Time::now();
  ClearVisualizationMsg(vis_msg_);
  visualization::DrawCross(odom_loc, 0.5, 0x00000000, vis_msg_);
  visualization::DrawLine(odom_loc,
                          Eigen::Rotation2Df(odom_angle)*Vector2f(2,0),
                          0xFF0000,
                          vis_msg_);
  visualization_publisher_.publish(vis_msg_);*/
  PublishVisualization();
}

void InitCallback(const amrl_msgs::Localization2DMsg& msg) {
  const Vector2f init_loc(msg.pose.x, msg.pose.y);
  const float init_angle = msg.pose.theta;
  const string map = msg.map;
  printf("Initialize: %s (%f,%f) %f\u00b0\n",
         map.c_str(),
         init_loc.x(),
         init_loc.y(),
         RadToDeg(init_angle));
  particle_filter_.Initialize(map, init_loc, init_angle);
  trajectory_points_.clear();
  dbg_init_loc_ = init_loc;
  dbg_init_angle_ = init_angle;
}

// OLEG TODO: remove DBG function maybe at the end
void debug_get_sub_seg_in_circ() {
  /*int i = 0;
  Vector2f point(1, 8);
  Vector2f center(5.6,8.2);
  float r = 5;
  cout << geometry::is_point_in_circle(center,r,point) << endl;
  point << 9.935,10.71;
  cout << geometry::is_point_in_circle(center,r,point) << endl;
  point << 5.6,4.2;
  cout << geometry::is_point_in_circle(center,r,point) << endl;*/

  //Line<float> segment(Vector2f(2,5),Vector2f(18,5));
  float r = 10;
  Vector2f center(12.0517320633,8.622621104584);
  Line<float> segment(Vector2f(10.2604,2.72384),Vector2f(3.33834,2.72384));
  Line<float> result = geometry::get_sub_segment_in_circle(segment, center,r);
  if (result.Length() < GenConsts::kEpsilon) {
      cout << "not inside." << endl;
  } else {
      cout << "part inside: ( " << result.p0.x() << "," << result.p0.y() << "),("
           << result.p1.x() << "," << result.p1.y() << ")" << endl;
  }

}

void debug_get_predicted_pt_cld() {


  static vector<Vector2f> scan;
  static vector<float> exp_ranges;
  //Vector2f loc(-7.93723344803,7.96144914627);
  //Vector2f loc(15.1750001907, 8.67500019073);
  particle_filter_.GetPredictedPointCloud(dbg_init_loc_, dbg_init_angle_,1000,0.2,10.0,
                                         -2.35619449615,
                                          2.35619449615,
                                          &scan, &exp_ranges);
  cout << "scan size:" << scan.size() << endl;
  /*for (const auto &p : scan) {
      cout << p.x()<< "," << p.y() << " ";
  }*/
  cout << endl;
  visualization::ClearVisualizationMsg(vis_msg_);
  visualization::DrawPointCloud(scan, 0xFF00, vis_msg_);
  visualization_publisher_.publish(vis_msg_);
}

void ProcessLive(ros::NodeHandle* n) {
  ros::Subscriber initial_pose_sub = n->subscribe(
      FLAGS_init_topic.c_str(),
      1,
      InitCallback);
  ros::Subscriber laser_sub = n->subscribe(
      FLAGS_laser_topic.c_str(),
      1,
      LaserCallback);
  ros::Subscriber odom_sub = n->subscribe(
      FLAGS_odom_topic.c_str(),
      1,
      OdometryCallback);
  while (ros::ok() && run_) {
    ros::spinOnce();
    PublishVisualization();
    Sleep(0.01);
  }
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "particle_filter", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  InitializeMsgs();
  //map_ = vector_map::VectorMap(CONFIG_map_name_);

  visualization_publisher_ =
      n.advertise<VisualizationMsg>("visualization", 1);
  localization_publisher_ =
      n.advertise<amrl_msgs::Localization2DMsg>("localization", 1);
  laser_publisher_ =
      n.advertise<sensor_msgs::LaserScan>("scan", 1);
  pfresult_publisher_ =
      n.advertise<std_msgs::String>("pf", 128);

  particle_filter::PfParams params;
  params.radar_downsample_rate = 20;
  params.num_particles = 30;
  params.resample_n_step= 5;
  params.d_long=0.3;
  params.d_short=0.3;
  particle_filter_.SetParams(params);
  particle_filter_.SetRosHandleAndInitPubs(&visualization_publisher_, &vis_msg_);

  ProcessLive(&n);

  return 0;
}
