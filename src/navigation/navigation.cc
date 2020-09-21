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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "xtensor/xarray.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor/xbuffer_adaptor.hpp"
#include "xtensor/xio.hpp"
#include "collision_detect_functions.h"
#include <vector>
#include <iostream>
#include "clock.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
    ros::Publisher drive_pub_;
    ros::Publisher viz_pub_;
    VisualizationMsg local_viz_msg_;
    VisualizationMsg global_viz_msg_;
    AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
    const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

    Navigation::Navigation(const string &map_file, ros::NodeHandle *n) :
        robot_loc_(0, 0),
        robot_angle_(0),
        robot_vel_(0, 0),
        robot_omega_(0),
        odom_angle_(0),
        nav_complete_(true),
        nav_goal_loc_(0, 0),
        nav_goal_angle_(0),
        plot_publisher_{n},
        robot_curvature_(0),
        robot_center_of_turning_(0, 0),
        robot_target_loc_(0, 0),
        is_initloc_inited_{false},
        world_(SamplingConsts::downsample_rate_space, SamplingConsts::downsample_rate_time),
        latency_tracker_{plot_publisher_, 0.05f},
        latency_size_{0},
        state_estimator_{PhysicsConsts::act_latency_portion*PhysicsConsts::default_latency,
                        (1-PhysicsConsts::act_latency_portion)*PhysicsConsts::default_latency,0},
                collision_planner_(world_) {
    drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
        "ackermann_curvature_drive", 1);
    viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
    local_viz_msg_ = visualization::NewVisualizationMessage(
        "base_link", "navigation_local");
    global_viz_msg_ = visualization::NewVisualizationMessage(
        "map", "navigation_global");
    InitRosHeader("base_link", &drive_msg_.header);
    Clock::now();
    }

    void Navigation::SetNavGoal(const Vector2f &loc, float angle) {
    }

    void Navigation::UpdateLocation(const Eigen::Vector2f &loc, float angle) {

    }

    void Navigation::UpdateOdometry(const Vector2f &loc,
                                    float angle,
                                    const Vector2f &vel,
                                    float ang_vel) {
        if (!is_initloc_inited_) {
            init_loc_ = robot_loc_;
            init_angle_ = driver.calculate_initial_angle(robot_angle_, robot_curvature_);
            // Oleg Question: what is wrong with loc zero, not sure I understand how this works.
            auto is_loc_finite = init_loc_.allFinite();
            auto is_loc_nonzero = !init_loc_.isZero();
            if (is_loc_finite and is_loc_nonzero) {
                is_initloc_inited_ = true;
                ROS_INFO("Init loc %f, %f", init_loc_[0], init_loc_[1]);
            }
            }
        robot_loc_ = loc;
        robot_angle_ = angle;
        robot_vel_ = vel;
        robot_omega_ = ang_vel;

        latency_tracker_.add_measurements(VelocityMeasurement{vel.norm(), ros::Time::now().toSec()});

        float curr_time = Clock::now();

        plot_publisher_.publish_named_point("Obsrv Dist", curr_time, (loc - init_loc_).norm());
        plot_publisher_.publish_named_point("Obsrv Spd", curr_time, vel.norm());
        }

    void Navigation::ObservePointCloud(const vector<Vector2f> &cloud,
                                    double time) {
        laser_pcloud_local_frame_ = cloud;
    }


    float Navigation::ComputeDis2Stop(){
        float latency = latency_tracker_.estimate_latency();
        float actuation_latency = latency * PhysicsConsts::act_latency_portion;
        float observation_latency = latency - actuation_latency;
        float curr_spd = robot_vel_.norm();     // current speed of robot based on odometry reading
        float dis2stop = curr_spd * actuation_latency;      // distance that the robot travels over a given latency time period

        /* starting at the current observed speed (odometry) minus the speed lossed over the time period of observation latency, until the current speed is less than 0,
            decrease the current speed by the amount of speed that can be lossed over a time period of observation latency

            The total distance to stop is the sum of distances that the car travels at each given speed observed during the course of complete deceleration
        */
        for (curr_spd -= PhysicsConsts::max_acc * observation_latency; curr_spd >= 0.0;
        curr_spd -= PhysicsConsts::max_acc * observation_latency)
            dis2stop += curr_spd * latency;


        plot_publisher_.publish_named_point("Est Ltcy", Clock::now(), latency);

        return dis2stop;
    }

    vector<Vector2f> Navigation::ConvertLaserCloudToOdomFrame() {
        vector<Vector2f> odom_frame_point_cloud;
        odom_frame_point_cloud.clear();
        for (size_t i = 0; i < laser_pcloud_local_frame_.size(); ++i) {
            Vector2f odom_frame_point((laser_pcloud_local_frame_[i] + robot_loc_));
            odom_frame_point_cloud.push_back(odom_frame_point);
        }

        return odom_frame_point_cloud;
    }


    void Navigation::Test() {
        // function for debugging. Fill you test code here and run it from navigation_main (instead of changing Run).
          std::cout << "This is a test..." << std::endl;

          float r=5;
          Vector2f circle(0,0);
          Vector2f point(1,1);
          std::cout << collision::is_point_in_circle(circle, r, point);

          Vector2f point1(0,4);
          std::cout << collision::is_point_in_circle(circle, r, point1);

          Vector2f point2(0,5.3);
          std::cout << collision::is_point_in_circle(circle, r, point2);

          Vector2f point3(4,4);
          std::cout << collision::is_point_in_circle(circle, r, point3) << std::endl;

          float c = 1/r;
          Vector2f point4(2.5,0.67);
          std::cout << collision::calc_distance_on_curve_to_point(c, point4) << std::endl;

          //float r_out=6;

          //is_point_in_path(0);

          //throw "Done.";
    }
    
    // Oleg TODO:: move this function to the collision planner.
    float Navigation::RePlanPath() {
            std::vector<float> candidate_curvatures = collision_planner_.generate_candidate_paths();
            float best_c = 0;
            float best_fpl = 0;
            std::cout << "RePlan  " << std::endl << "--------------";
            for (size_t i = 0; i < candidate_curvatures.size(); ++i) {
                float candidate = candidate_curvatures[i];
                std::cout << "    candidate:  " << candidate << std::endl;
                auto colliding_points =
                        collision_planner_.select_potential_collision(candidate, laser_pcloud_local_frame_);
                if (colliding_points.empty()) {
                    std::cout << "    colliding points empty." << std::endl;
                    best_c = candidate;
                    break;
                }
                else {
                    float fpl;
                    if (fabs(candidate) < GenConsts::kEpsilon) {
                        fpl = collision::check_collision_curvature_zero(laser_pcloud_local_frame_);
                        std::cout << "    C==0" << std::endl;

                    } else {
                        float angle = collision_planner_.calculate_shortest_collision(candidate, colliding_points);
                        fpl = (1/candidate)*angle;
                    }
                    if (fabs(fpl-10) < 0.5) {
                        if (fabs(candidate) < GenConsts::kEpsilon) {
                            std::cout << "Curve Zero is clear."  << std::endl;
                        }
                        best_c = candidate;
                        break;
                    }
                    std::cout << "       FPL:  " << fpl << std::endl;
                    if (fpl > best_fpl) {
                        std::cout << "       Setting as best candidate." << fpl << std::endl;
                        best_fpl = fpl;
                       best_c = candidate;
                    }
                }
            }
            std::cout << std::endl  << std::endl <<  std::endl;
            drive_msg_.curvature = best_c;
            return best_c;
        }



    /*float Navigation::RePlanPath_Test() {
        std::vector<float> candidate_curvatures = collision_planner_.generate_candidate_paths();
        float best_c = 0;
        float best_fpl =  std::numeric_limits<float>::infinity();
        for (size_t i = 0; i < candidate_curvatures.size(); ++i) {
            float candidate = candidate_curvatures[i];
            auto colliding_points =
                    collision_planner_.select_potential_collision(candidate, laser_pcloud_local_frame_);
            if (colliding_points.empty()) {
                best_c = candidate;
                break;
            }
            else {
                float angle = collision_planner_.calculate_shortest_collision(candidate, colliding_points);
                float fpl = collision_planner_.calc_dist_on_curve_for_angle(candidate, angle);
                if (fpl < best_fpl) {
                   best_fpl = fpl;
                   best_c = candidate;
                }
            }
        }
        drive_msg_.curvature = best_c;
        return best_c;
    }*/

    float Navigation::SetOptimalVelocity(float target_dist) {
        float c_p = 0.01f;
        float epsilon = 0.005f;
        auto spd_inc = latency_tracker_.estimate_latency() * PhysicsConsts::max_acc;

        // Distance to stop {Vt^2 - V0^2 = 2ad}
        // auto v0_norm = robot_vel_.norm();
        // auto dis2stop = (0.0f - v0_norm * v0_norm) / (2.0f * -PhysicsConsts::max_acc) + epsilon;
        float dis2stop = ComputeDis2Stop() + epsilon;

        // curvature determined by collision planner 
        float desired_curvature = RePlanPath();
        robot_curvature_ = RePlanPath();      
        
        // curvature determined by collision planner
        float arc_length_distance = target_dist;   
        
        // if the curvature changes, a new "target location" is created. The center of turning will need to be updated.
        if (desired_curvature != robot_curvature_){
            init_angle_ = driver.calculate_initial_angle(robot_angle_, robot_curvature_);
        }
        
        robot_center_of_turning_ = driver.get_center_of_turning(robot_curvature_, robot_loc_, robot_angle_);
        robot_target_loc_ = driver.calculate_target_location(arc_length_distance, robot_curvature_, init_angle_, robot_center_of_turning_);
        
        // distance until end of arc path
        auto curr_dist = driver.calculate_current_distance(robot_curvature_, robot_loc_, robot_target_loc_, arc_length_distance);
        auto curr_spd = robot_vel_.norm();

        // update current speed
        driver.update_current_speed(dis2stop, is_initloc_inited_, curr_spd, curr_dist, spd_inc, c_p);
        
        drive_msg_.velocity = driver.get_velocity();
        drive_msg_.velocity = driver.drive_msg_check(drive_msg_.velocity);

        return drive_msg_.velocity;
    }
         
    void Navigation::Run() {
        // Create Helper functions here
        // Milestone 1 will fill out part of this class.
        // Milestone 3 will complete the rest of navigation.

        // Speed increment
        /*
        auto a = xt::adapt(robot_loc_.data(), 2, xt::no_ownership(), std::vector<int>{2});
        std::cout << a << std::endl;
         */

        //Test();
        RePlanPath();
        SetOptimalVelocity(4);

        drive_pub_.publish(drive_msg_);

        //printf("Latency %.2f, latency samples %ld\n", latency_tracker).estimate_latency(), latency_tracker_.get_alllatencies().size());
        double curr_time = ros::Time::now().toSec();
        latency_tracker_.add_controls(VelocityControlCommand{drive_msg_.velocity, curr_time});
        plot_publisher_.publish_named_point("Ctrl cmd vel", Clock::now(), drive_msg_.velocity);
        plot_publisher_.publish_named_point("Ctrl cmd c", Clock::now(), drive_msg_.curvature);

        /*float c_p = 0.01f;
        float epsilon = 0.005f;
        auto spd_inc = latency_tracker_.estimate_latency() * PhysicsConsts::max_acc;
        // Distance to stop {Vt^2 - V0^2 = 2ad}
        // auto v0_norm = robot_vel_.norm();
        // auto dis2stop = (0.0f - v0_norm * v0_norm) / (2.0f * -PhysicsConsts::max_acc) + epsilon;
        float dis2stop = ComputeDis2Stop() + epsilon;
        // Current Distance
        auto curr_dist = (robot_loc_ - init_loc_).norm();
        auto curr_spd = robot_vel_.norm();
        if (curr_dist >= Assignment0::target_dis or !is_initloc_inited_) {
            drive_msg_.velocity = 0;
        } else if (curr_dist + dis2stop >= Assignment0::target_dis) {
            curr_spd -= spd_inc + (Assignment0::target_dis - curr_dist) * c_p;
            drive_msg_.velocity = curr_spd;
        } else if (curr_spd >= PhysicsConsts::max_vel) {
            curr_spd = PhysicsConsts::max_vel;
            drive_msg_.velocity = curr_spd;
        } else {
            curr_spd += spd_inc;
            drive_msg_.velocity = curr_spd;
        }
        drive_msg_.velocity = drive_msg_.velocity > 1.0 ? 1.0 : drive_msg_.velocity;
        drive_msg_.velocity = drive_msg_.velocity < 0.0 ? 0.0 : drive_msg_.velocity;
        drive_msg_.velocity = 0.0f;
        drive_pub_.publish(drive_msg_);
        double curr_time = ros::Time::now().toSec();
        //printf("Latency %.2f, latency samples %ld\n", latency_tracker).estimate_latency(), latency_tracker_.get_alllatencies().size());
        //latency_tracker_.add_controls(VelocityControlCommand{drive_msg_.velocity, curr_time});
        //plot_publisher_.publish_named_point("Ctrl cmd", Clock::now(), drive_msg_.velocity);
        */

        /*
        visualization::ClearVisualizationMsg(local_viz_msg_);
        float curvature = 0.5f, extending_radius = 5.f;
        //float offset_omega = curvature > 0 ? (-M_PI / 2.f) : (M_PI / 2.f);
        CollisionPlanner cp{world_};
        auto corner_params = cp.convert4corner2cspace(curvature);
        for (int i = 0; i < 4; ++i) {
            visualization::DrawArc(Vector2f{0, 1.0 / curvature}, corner_params(i, 0), corner_params(i, 1),
                                   corner_params(i, 1) + extending_radius, 0x0000FFFF, local_viz_msg_);
        }
        auto w = CarDims::w + CarDims::default_safety_margin * 2;
        auto l = CarDims::l + CarDims::default_safety_margin * 2;
        Vector2f corner1{(l + CarDims::wheelbase) / 2.f, w / 2.f};
        Vector2f corner2{(l + CarDims::wheelbase) / 2.f, -w / 2.f};
        Vector2f corner3{-(l - CarDims::wheelbase) / 2.f, -w / 2.f};
        Vector2f corner4{-(l - CarDims::wheelbase) / 2.f, w / 2.f};
        visualization::DrawLine(corner1, corner2, 0x000000FF, local_viz_msg_);
        visualization::DrawLine(corner2, corner3, 0x000000FF, local_viz_msg_);
        visualization::DrawLine(corner3, corner4, 0x000000FF, local_viz_msg_);
        visualization::DrawLine(corner4, corner1, 0x000000FF, local_viz_msg_);
        if (!laser_pcloud_local_frame_.empty()) {
            auto coll = cp.select_potential_collision(curvature, laser_pcloud_local_frame_);
           //for (auto &pt : coll) {
                //visualization::DrawPoint(Vector2f{pt[0], pt[1]}, 0x00FF00FF, local_viz_msg_);
           //     float start = fmin(offset_omega, pt(1));
           //     float end = fmax(offset_omega, pt(1));
           //     visualization::DrawArc(Vector2f{0, 1.0 / curvature}, pt(0), start, end, 0x00FF00FF, local_viz_msg_);
           // }
           // auto steps = 10;
           // auto pt1 = coll.front();
           // auto pt2 = coll.back();
           // auto inc = (pt2[0] - pt1[0]) / steps;
           // for (int i = 0; i < steps; ++i){
           //     float rp = pt1[0] + i * inc;
           //     float thetap = cp.linear_interpolate_params_wrt_R(pt1, pt2, rp);
           //     float start = fmin(offset_omega, thetap);
           //     float end = fmax(offset_omega, thetap);
           //     visualization::DrawArc(Vector2f{0, 1.0 / curvature}, rp, start, end, 0x00FF00FF, local_viz_msg_);
           // }
            float clearance = cp.calculate_shortest_collision(curvature, coll);
            ROS_INFO("clearance %f", clearance);
            for (int i = 0; i < 4; ++i) {
                float start = fmin(corner_params(i, 1), corner_params(i, 1) + clearance);
                float end = fmax(corner_params(i, 1), corner_params(i, 1) + clearance);
                visualization::DrawArc(Vector2f{0, 1.0 / curvature}, corner_params(i, 0), start, end,
                                       0x00FF00FF, local_viz_msg_);
            }
            viz_pub_.publish(local_viz_msg_);
        }*/

    }
}  // namespace navigation
