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
\file    navigation_main.cc
\brief   Main entry point for reference Navigation implementation
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>
#include <algorithm>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "amrl_msgs/Localization2DMsg.h" 

#include "navigation.h"
#include "global_planner.h"
#include "shared/math/geometry.h"

// OLEG TODO remove this include it is for debug:
#include "vector_map/vector_map.h"
#include "amrl_msgs/VisualizationMsg.h"
using amrl_msgs::VisualizationMsg;


using amrl_msgs::Localization2DMsg;
using math_util::DegToRad;
using math_util::RadToDeg;
using navigation::Navigation;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using std::string;
using std::vector;
using Eigen::Vector2f;

// Create command line arguments
DEFINE_string(laser_topic, "scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "odom", "Name of ROS topic for odometry data");
DEFINE_string(loc_topic, "localization", "Name of ROS topic for localization");
DEFINE_string(init_topic,
              "initialpose",
              "Name of ROS topic for initialization");
DEFINE_string(map, "maps/GDC1/GDC1.vectormap.txt", "Name of vector map file");

bool run_ = true;
sensor_msgs::LaserScan last_laser_msg_;
Navigation* navigation_ = nullptr;

ros::Publisher visualization_pub_;
VisualizationMsg map_viz_msg_;

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f, dt=%f\n",
           msg.header.stamp.toSec(),
           GetWallTime() - msg.header.stamp.toSec());
  }
  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(0.2, 0);
  static vector<Vector2f> point_cloud_;

  // OLEG TODO: techincally, to make it work faster, don't clear it, when size==0, init with laser results size 
  // with some overhead, otherwise just go by indexes and set new value, if point_cloud_ size is bigger then 
  // msg.ranges.size() set all the remaining vector indexes to zero.

  tf::proj_lidar_2_pts(msg, point_cloud_, kLaserLoc, 1);
  navigation_->ObservePointCloud(point_cloud_, msg.header.stamp.toSec());
  last_laser_msg_ = msg;

}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  navigation_->UpdateOdometry(
      Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y),
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
      Vector2f(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
      msg.twist.twist.angular.z);
}

void GoToCallback(const geometry_msgs::PoseStamped& msg) {
  const Vector2f loc(msg.pose.position.x, msg.pose.position.y);
  const float angle =
      2.0 * atan2(msg.pose.orientation.z, msg.pose.orientation.w);
  printf("Goal: (%f,%f) %f\u00b0\n", loc.x(), loc.y(), angle);
  navigation_->SetNavGoal(loc, angle);
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

void LocalizationCallback(const amrl_msgs::Localization2DMsg msg) {
  if (FLAGS_v > 0) {
    printf("Localization t=%f\n", GetWallTime());
  }
  printf("Localize %f %f %f\n", msg.pose.x, msg.pose.y, msg.pose.theta);
  navigation_->UpdateLocation(Vector2f(msg.pose.x, msg.pose.y), msg.pose.theta);
}

planning::Graph test() {
    vector_map::VectorMap map;
    string map_file = "GDC1";
    string full_map_file;
    if (map_file.find('.') == string::npos) {
       full_map_file = "maps/" + map_file + "/" + map_file + ".vectormap.txt";
    } else {
       full_map_file = map_file;
    }

    cout << "Loading map file '" << full_map_file << "'...." << endl;
    map.Load(full_map_file);
    cout << "Done loading map." << endl;

    float  margin_to_wall = 0.10;
    float grid_spacing = 1;
    int orient_num = 1;
    planning::Graph graph(grid_spacing, -50, 50, -30, 30, orient_num , margin_to_wall, map);


    //graph.GenerateGraph(map);

    /*planning::Vertices V = graph.GetVertices();
    std::cout << "X dim:" << V.size() << std::endl;
    for (auto &y_vector: V) {
        std::cout << "Y_dim" << y_vector.size()  << std::endl;
        for (auto &o_vector: y_vector) {
            std::cout << "orient_dim" << o_vector.size() << std::endl;
            break;
        }

    }*/

    //V[6][2][1].push_back(planning::GraphIndex(7,2,1));
    //V[6][2][1].push_back(planning::GraphIndex(7,3,1));

    planning::GraphIndex index = graph.GetClosestVertex(PoseSE2(0, 25, -7.15));
    cout << "X id:" << index.x << " Y id:" << index.y << " Orient id:"
         << index.orient << std::endl;

    return graph;

}

void test_geomutils() {
    float r = 2;
    Eigen::Vector2f center(0,2);
    Eigen::Vector2f p(0,4);
    bool is_end_point;
    float dist = geometry::dist_arc_point(center, r, p, -M_PI/2, M_PI, is_end_point);
    cout << "dist:" << dist << endl;

    Vector2f a(0,0);
    Vector2f b(4,0);
    p = Vector2f(6,3);
    cout << "prj:" << geometry::ProjectPointOntoLineSegment(p,a,b) << endl;
    cout << "point:" << geometry::dist_line_point(a,b,p, is_end_point) << endl;
    cout << "is end:" << is_end_point << endl;

}


std::list<planning::GraphIndex> navtest(planning::Graph graph){ 
  PoseSE2 start(-25, 6, 0);
  //start.loc = robot_loc_;
  //start.angle = robot_angle_;
  PoseSE2 goal(35, 12, 0);
  //goal.loc = nav_goal_loc_;
  //goal.angle = nav_goal_angle_;

  planning::A_star gplan(graph);

  return gplan.generatePath(start, goal);
}

void visualizeGraph(planning::Graph graph){

    planning::Vertices V = graph.GetVertices();
    visualization::ClearVisualizationMsg(map_viz_msg_);

    //visualization::DrawCross(Eigen::Vector2f(0,0), 0.5, 0x000FF, map_viz_msg_);
    for (int x_id = 0; x_id < (int)V.size(); ++x_id) {
        for (int y_id = 0; y_id < (int)V[0].size(); ++y_id) {
            Eigen::Vector2f vertex_loc = graph.GetLocFromVertexIndex(x_id,y_id);
            for (int o_id = 0; o_id < (int)V[0][0].size(); ++o_id) {
                planning::GraphIndex curr_vertex(x_id,y_id,o_id);
                list<planning::GraphIndex> neighbors = graph.GetVertexNeighbors(curr_vertex);
                for ( auto &neighbor : neighbors) {
                    //geometry::line2f edge(
                    //        graph.GetLocFromVertexIndex(x_id,y_id),
                    //       graph.GetLocFromVertexIndex(neighbor.x,neighbor.y));
                    visualization::DrawLine(
                            graph.GetLocFromVertexIndex(x_id,y_id),
                            graph.GetLocFromVertexIndex(neighbor.x,neighbor.y),
                            0x0000000,
                            map_viz_msg_);
                }
            }

            visualization::DrawCross(vertex_loc, 0.25, 0x000FF, map_viz_msg_);

        }
    }
    visualization_pub_.publish(map_viz_msg_);
}

void visualizePath(planning::Graph graph, std::list<planning::GraphIndex> plan){
  for(const auto& node : plan)
  {
    Eigen::Vector2f node_loc = graph.GetLocFromVertexIndex(node.x,node.y);
    std::cout << "[" << node.x << " " << node.y << "] ";
    visualization::DrawCross(node_loc, 0.25, 0x000FF, map_viz_msg_);
  }
  std::cout << std::endl;
  visualization_pub_.publish(map_viz_msg_);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "navigation", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;

  navigation::NavParams params;
  params.plan_grid_pitch = 0.5;
  params.plan_x_start = -160;
  params.plan_x_end = 160;
  params.plan_y_start = -160;
  params.plan_y_end = 160;
  params.plan_num_of_orient = 1;
  params.plan_margin_to_wall = 0.3;
  params.replan_dist = 2.5;
  params.pure_pursuit_circ_rad = 1.1;
  params.obs_min_clearance = 0.1;

    params.plan_grid_pitch = 0.3;
    params.plan_x_start = -50;
    params.plan_x_end = 50;
    params.plan_y_start = -50;
    params.plan_y_end = 50;
    params.plan_num_of_orient = 1;
    params.plan_margin_to_wall = 0.5;
    params.replan_dist = 2.5;
    params.pure_pursuit_circ_rad = 0.8;
    params.obs_min_clearance = 0.1;

  navigation_ = new Navigation(FLAGS_map, params, &n);

  ros::Subscriber velocity_sub =
      n.subscribe(FLAGS_odom_topic, 1, &OdometryCallback);
  ros::Subscriber localization_sub =
      n.subscribe(FLAGS_loc_topic, 1, &LocalizationCallback);
  ros::Subscriber laser_sub =
      n.subscribe(FLAGS_laser_topic, 1, &LaserCallback);
  ros::Subscriber goto_sub =
      n.subscribe("/move_base_simple/goal", 1, &GoToCallback);
  visualization_pub_ =
      n.advertise<VisualizationMsg>("visualization", 1);
  map_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation");


  RateLoop loop(20.0);
  //planning::Graph graph = test();
  //std::list<planning::GraphIndex> Astar = navtest(graph);
  //test_geomutils();
  while (run_ && ros::ok()) {
    ros::spinOnce();
    navigation_->Run();
    //visualizeGraph(graph, Astar);
    //visualizePath(graph,Astar);
    //std::cout << "Astar size: " << Astar.size() << std::endl;
    loop.Sleep();

  }
  delete navigation_;
  return 0;
}
