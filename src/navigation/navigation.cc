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
#include <vector>
#include <iostream>
#include "clock.h"
#include "shared/global_utils.h"


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

    Navigation::Navigation(const string &map_file, NavParams params, ros::NodeHandle *n) :
            robot_loc_(0, 0),
            robot_angle_(0),
            robot_vel_(0, 0),
            robot_omega_(0),
            odom_loc_(0),
            odom_angle_(0),
            nav_complete_(true),
            nav_goal_loc_(0, 0),
            nav_goal_angle_(0),
            plot_publisher_{n},
            is_initloc_inited_{false},
            step_num_{0},
            world_(SamplingConsts::downsample_rate_space, SamplingConsts::downsample_rate_time),
            latency_tracker_{plot_publisher_, 0.05f},
            latency_size_{0},
            state_estimator_{PhysicsConsts::act_latency_portion*PhysicsConsts::default_latency,
                            (1-PhysicsConsts::act_latency_portion)*PhysicsConsts::default_latency,0},
            collision_planner_(world_),
            nav_params_(params) {
        drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
                "ackermann_curvature_drive", 1);
        viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
        local_viz_msg_ = visualization::NewVisualizationMessage(
                "base_link", "navigation_local");
        global_viz_msg_ = visualization::NewVisualizationMessage(
                "map", "navigation_global");
        InitRosHeader("base_link", &drive_msg_.header);
        Clock::now();

        vector_map::VectorMap map = LoadMap(map_file);
        graph_ = Graph(params.plan_grid_pitch,
                              params.plan_x_start,
                              params.plan_x_end,
                              params.plan_y_start,
                              params.plan_y_end,
                              params.plan_num_of_orient,
                              params.plan_margin_to_wall,
                              map);

        glob_planner_ = A_star(graph_);
    }

    vector_map::VectorMap Navigation::LoadMap(const std::string& map_file) {
        string full_map_file;
        vector_map::VectorMap map;

        if (map_file.find('.') == string::npos) {
           full_map_file = "maps/" + map_file + "/" + map_file + ".vectormap.txt";
        } else {
           full_map_file = map_file;
        }

        cout << "Loading map file '" << full_map_file << "'...." << endl;
        map.Load(full_map_file);
        cout << "Done loading map." << endl;\
        return map;

    }

    void Navigation::visPlan() {
        for(const auto& node : plan_) {
            Eigen::Vector2f node_loc = graph_.GetLocFromVertexIndex(node.x,node.y);
            //std::cout << "[" << node_loc.x() << " " << node_loc.y() << "] ";
            visualization::DrawCross(node_loc, 0.15, 0x000FF, global_viz_msg_);
        }
        //cout << endl;
    }


    void Navigation::SetNavGoal(const Vector2f &loc, float angle) {
            nav_goal_loc_ = loc;
            nav_goal_angle_ = angle;
            nav_complete_ = false;

            cout << "Set Nav Goal" << endl << endl;


            Eigen::Vector2f carrot_loc;
            PoseSE2 start(robot_loc_.x(),robot_loc_.y(),0);
            PoseSE2 goal(nav_goal_loc_.x(),nav_goal_loc_.y(),0);
            plan_ = glob_planner_.generatePath(start, goal);
            bool intersects = glob_planner_.getPurePursuitCarrot(
                    robot_loc_,
                    nav_params_.pure_pursuit_circ_rad,
                    carrot_loc);
            cout << " Pure Pursuit Done." << endl;
            visualization::ClearVisualizationMsg(global_viz_msg_);
            if (intersects) {
              visualization::DrawCross(carrot_loc, 0.5, 0xFF0000, global_viz_msg_);
              cout << "\"Carrot\" found..." << endl;
            }

            visPlan();


        }

    void Navigation::UpdateLocation(const Eigen::Vector2f &loc, float angle) {
        debug::print_loc(loc," \n\nUpdateLocation::Loc");
        robot_loc_ = loc;
        robot_angle_ = angle;
    }

    void Navigation::UpdateOdometry(const Vector2f &loc,
                                    float angle,
                                    const Vector2f &vel,
                                    float ang_vel) {
        if (!is_initloc_inited_) {
            init_loc_ = robot_loc_;
            // Oleg Question: what is wrong with loc zero, not sure I understand how this works.
            auto is_loc_finite = init_loc_.allFinite();
            auto is_loc_nonzero = !init_loc_.isZero();
            if (is_loc_finite and is_loc_nonzero) {
                is_initloc_inited_ = true;
                ROS_INFO("Init loc %f, %f", init_loc_[0], init_loc_[1]);
            }
        }
        odom_loc_ = loc;
        odom_angle_ = angle;
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

    float Navigation::ComputeDis2Stop() {
        float latency = PhysicsConsts::default_latency;
        float actuation_latency = PhysicsConsts::default_latency * PhysicsConsts::act_latency_portion;
        float observation_latency = latency - actuation_latency;
        float curr_spd = robot_vel_.norm();
        float dis2stop = curr_spd * actuation_latency;
        std::cout << "  Actuation Latency: " << actuation_latency << std::endl;

        for (curr_spd -= PhysicsConsts::max_acc * observation_latency; curr_spd >= 0.0;
             curr_spd -= PhysicsConsts::max_acc * observation_latency)
            dis2stop += curr_spd * latency;


        plot_publisher_.publish_named_point("Est Ltcy", Clock::now(), latency);

        return dis2stop;
    }

    vector<Vector2f> Navigation::ConvertLaserCloudToOdomFrame() {
        vector<Vector2f> odom_frame_point_cloud;
        //odom_frame_point_cloud.clear();
        for (size_t i = 0; i < laser_pcloud_local_frame_.size(); ++i) {
            Vector2f odom_frame_point =
                tf::transform_point_to_glob_frame(
                    PoseSE2(odom_loc_,odom_angle_),
                    laser_pcloud_local_frame_[i]);
            odom_frame_point_cloud.push_back(odom_frame_point);
        }

        return odom_frame_point_cloud;
    }

    vector<Vector2f> Navigation::ConvertLaserCloudToOtherFrame(const Vector2f& loc,float angle) {
            static vector<Vector2f> new_frame_point_cloud;
            new_frame_point_cloud.clear();
            for (size_t i = 0; i < laser_pcloud_local_frame_.size(); ++i) {
                Eigen::Rotation2Df rotation(angle);
                Vector2f new_frame_point(rotation*(laser_pcloud_local_frame_[i] + loc));
                new_frame_point_cloud.push_back(new_frame_point);
            }

            return new_frame_point_cloud;
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
    
    float Navigation::FindMinClearance(float c, float arc_length,
                                       const std::vector<Vector2f>& work_point_cloud) {
        //cout << "FindMinClearance::"<< endl;
        if (arc_length < 0)
            arc_length = -arc_length;

        float min_dist = std::numeric_limits<float>::infinity();
        bool is_end_point;
        if (fabs(c) < GenConsts::kEpsilon) {
            Vector2f l0(0,0);
            Vector2f l1(arc_length,0);
            for (const auto& p: work_point_cloud) {
                float point_dist = geometry::dist_line_point(
                        l0,
                        l1,
                        p,
                        is_end_point);

                if (point_dist < min_dist && point_dist >= GenConsts::kEpsilon) {
                    min_dist = point_dist;
                }
            }
        } else {
            float r = fabs(1/c);
            if (arc_length > 2*M_PI*r)
                arc_length = 2*M_PI*r;

            Vector2f center;
            float end_angle, start_angle;
            if (c > 0) {
                center = Vector2f(0,r);
                start_angle = -M_PI/2;
                end_angle = -M_PI/2 + arc_length/r;
            } else {
                center = Vector2f(0,-r);
                end_angle = M_PI/2;
                start_angle = M_PI/2 - arc_length/r;
            }

            /*cout << "arc length:" << arc_length
                 << " start_angle" << start_angle
                 << " end_angle " << end_angle
                 << " r" << r ;
            debug::print_loc(center,"  center",true);
            //debug::print_loc(,"  center",false);
            cout << "p cloud size:" << work_point_cloud.size() << endl;*/
            int i = 0;
            for (const auto& p: work_point_cloud) {
                //cout << "i: " << i;
                //debug::print_loc(p,"  point",true);
                float point_dist = geometry::dist_arc_point(
                        center, r, p,
                        start_angle,
                        end_angle,
                        is_end_point);
                if (point_dist < min_dist && point_dist >= GenConsts::kEpsilon) {
                    min_dist = point_dist;
                }
                i++;
            }

        }
        //cout << "done loop" << endl;
        if ((min_dist < nav_params_.obs_min_clearance +
             CarDims::l/2 + CarDims::default_safety_margin) &&
                 min_dist > GenConsts::kEpsilon )
            return -100;
        return min_dist;
    }

    float Navigation::FindDistToGoal(float c, float arc_length,
                         Vector2f goal_loc, bool count_boundary_points) {
        //cout << "FindDistToGoal::"<< endl;
        bool is_end_point;
        float goal_dist;
        if (arc_length < 0)
            arc_length = -arc_length;
        if (fabs(c) < GenConsts::kEpsilon) {
            Vector2f l0(0,0);
            Vector2f l1(arc_length,0);
            goal_dist = geometry::dist_line_point(
                        l0, l1,goal_loc,
                        is_end_point);

        } else {
            float r = fabs(1/c);
            if (arc_length > 2*M_PI*r)
               arc_length = 2*M_PI*r;

            Vector2f center;
            float end_angle, start_angle;
            if (c > 0) {
               center = Vector2f(0,r);
               start_angle = -M_PI/2;
               end_angle = -M_PI/2 + arc_length/r;
            } else {
               center = Vector2f(0,-r);
               end_angle = M_PI/2;
               start_angle = M_PI/2 - arc_length/r;
            }
            //cout << "arc length:" << arc_length
            //                 << " start_angle" << start_angle
            //                 << " end_angle " << end_angle
            //                 << " r" << r ;
            //debug::print_loc(center,"  center",false);
            //debug::print_loc(goal_loc,"  goal_loc",true);

            goal_dist = geometry::dist_arc_point(
                       center, r, goal_loc,
                       start_angle,
                       end_angle,
                       is_end_point);
        }
        //cout << "done if else" << endl;
        if (is_end_point && !count_boundary_points) {
            goal_dist = 100;
        } else if (goal_dist < 0.5) {
            goal_dist = -100;
        }

        //cout << "done FindGoalDist" << endl;
        return goal_dist;
    }

    // Oleg TODO:: move this function to the collision planner.
    Eigen::Vector2f Navigation::PlanLocalPath(Vector2f goal_loc) {
            Eigen::Vector2f local_path;
            std::vector<float> candidate_curvatures = collision_planner_.generate_candidate_paths(0.01,1);
            float w1 = 1, w2 = 3, w3 =-4;
            float max_fpl = PhysicsConsts::radar_max_range-2;
            float best_c = 0;
            float best_fpl = 0;
            float best_reward = 0;
            std::cout << "PlanLocalPath  " << std::endl << "--------------" << std::endl;
            std::vector<Vector2f> work_point_cloud = laser_pcloud_local_frame_;
            /*std::vector<Vector2f> work_point_cloud;
            state_estimator_.transform_p_cloud_tf_obs_to_act(
                            laser_pcloud_local_frame_,
                            work_point_cloud);*/
            //std::cout << "Number of paths to go check:" <<  candidate_curvatures.size() << std::endl;
            //std::cout << "Max corvature size:" << candidate_curvatures[candidate_curvatures.size()-1] << std::endl;
            for (size_t i = 0; i < candidate_curvatures.size(); ++i) {
                float candidate = candidate_curvatures[i];
                float reward, clearance, dist_to_goal, fpl;
                //std::cout << "    RePlan::candidate:  " << candidate << std::endl;
                auto colliding_points =
                        collision_planner_.select_potential_collision(candidate, work_point_cloud);
                if (colliding_points.empty()) {
                    //No collision found. i.e FPL is the max range of the radar;
                    //std::cout << "    RePlan::colliding points empty." << std::endl;
                    clearance = FindMinClearance(candidate, max_fpl, work_point_cloud);
                    dist_to_goal = FindDistToGoal(candidate, max_fpl, goal_loc, true);
                    fpl = max_fpl;
                }
                else {
                    if (fabs(candidate) < GenConsts::kEpsilon) {
                        // Straigh Line Case
                        fpl = collision_planner_.calculate_shortest_collision_flat(work_point_cloud);
                        fpl = std::min(fpl,max_fpl);
                        //std::cout << "    RePlan::For C==0 FPL is " << fpl << std::endl;
                        clearance = FindMinClearance(candidate, fpl, work_point_cloud);
                        dist_to_goal = FindDistToGoal(candidate,fpl, goal_loc, false);
                    } else {
                        float angle = collision_planner_.calculate_shortest_collision(candidate, colliding_points);
                        fpl = (1/candidate)*angle;
                        fpl = std::min(fpl,max_fpl);
                        //std::cout << "    RePlan::Anngle " << angle << " FPL "<< fpl << std::endl;
                        clearance = FindMinClearance(candidate, fpl, work_point_cloud);
                        dist_to_goal = FindDistToGoal(candidate,fpl, goal_loc, true);
                        /*if (candidate > 0) {
                            visualization::DrawArc(Vector2f(0,1/candidate), std::abs(1/candidate),0,
                                                   angle,0x99CCFF,local_viz_msg_);
                        } else {
                            visualization::DrawArc(Vector2f(0,-1/candidate), std::abs(1/candidate),0,
                                                                               angle,0x99CCFF,local_viz_msg_);
                        }*/
                    }
                }
                //cout << "--> Reward fpl " << fpl << " clearance " << clearance << " dist_to_goal " << dist_to_goal << endl;

                reward = w1*fpl + w2*clearance + w3*dist_to_goal;
                //cout << "reward:" << reward << endl;
                /*if ((candidate == 0 && fabs(fpl-PhysicsConsts::radar_max_range) <
                    PhysicsConsts::radar_noise_std) || std::isinf(fpl)) {
                    if (fabs(candidate) < GenConsts::kEpsilon) {
                        std::cout << "  RePlan::Curve Zero is clear."  << std::endl;
                    }
                    best_c = candidate;
                    std::cout << "    RePlan:: Breaking after finding a path without any obstacles due to Radar Max Range ";
                    best_fpl = fpl;
                    collision_planner_.calculate_shortest_collision(candidate, colliding_points, local_viz_msg_);
                    break;
                }*/
                //td::cout << "       FPL:  " << fpl << std::endl;
                if (reward > best_reward) {
                    std::cout << "       Setting as best candidate. C: " << candidate << "FPL:" << fpl << std::endl;
                    best_fpl = fpl;
                    best_c = candidate;
                    best_reward = reward;
                }

            }

            std::cout << "    RePlan::Selected Curvature: " << best_c
                      << "  Best FPL:" << best_fpl << std::endl;
            //std::cout << std::endl  << std::endl <<  std::endl;
            drive_msg_.curvature = best_c;
            /*if (best_c > 0)
                visualization::DrawArc(Vector2f(0,1/best_c), std::abs(1/best_c),0,
                                   2.35,0xFF0000,local_viz_msg_);
            else if(best_c < 0)
                visualization::DrawArc(Vector2f(0,-1/best_c), std::abs(1/best_c),0,
                                           2.35,0xFF0000,local_viz_msg_);*/
            if (best_c == 0) {
                visualization::DrawLine(Vector2f(0,0),Vector2f(4,0),0xFF0000,local_viz_msg_);
            } else {
                visualization::DrawPathOption(best_c, best_fpl, 0, local_viz_msg_);

            }

            //visualization::DrawPathOption(best_c, best_fpl,  1, local_viz_msg_);
            local_path.x() = best_fpl;
            local_path.y() = best_c;
            return local_path;
        }



    float Navigation::SetOptimalVelocity(float target_dist, float curvature) {
        float c_p = 0.01f;
        float epsilon = 0.005f;
        //float actuation_latency = PhysicsConsts::act_latency_portion*PhysicsConsts::default_latency;
        auto spd_inc = PhysicsConsts::default_latency * PhysicsConsts::max_acc;
        float dis2stop = ComputeDis2Stop() + epsilon;
        auto curr_spd = robot_vel_.norm();

        // Update the distance traveled. If the target distance changed, the distance traveled would be reset
        driver.update_dist_traveled(curr_spd, PhysicsConsts::default_latency, target_dist, curvature);

        // update current speed
        driver.update_current_speed(dis2stop, spd_inc, is_initloc_inited_, curr_spd, c_p);
        
        drive_msg_.velocity = driver.get_new_velocity();
        drive_msg_.velocity = driver.drive_msg_check(drive_msg_.velocity);
        //drive_msg_.velocity = 1.0;
        std::cout << "Velocity: " << drive_msg_.velocity << std::endl;
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
        //viz_pub_.publish(global_viz_msg_);

        if (!is_initloc_inited_ || nav_complete_ )
            return;
        //static double start_timestamp;
        //if (step_num_ == 0) {
        //   start_timestamp = ros::Time::now().toSec();
        //}

        //double timestamp = ros::Time::now().toSec() - start_timestamp;

        visualization::ClearVisualizationMsg(local_viz_msg_);
        visualization::ClearVisualizationMsg(global_viz_msg_);
        //state_estimator_.update_estimation(robot_loc_,robot_angle_,timestamp);

        //state_estimator_.update_estimation(Vector2f(0,0),0, timestamp);
        //estimate_pose_local_frame_ =
        //    state_estimator_.estimate_state_cmd_actuation_time(timestamp);

        //visualization::DrawCross(estimate_pose_local_frame_.loc, 0.5, 0x00000000, local_viz_msg_);
        //visualization::DrawPoint(Vector2f(0,0),10, local_viz_msg_);

        //std::stringstream dbg_msg;
        //dbg_msg << " -- pose_queue_size = "
        //        << state_estimator_.get_pose_estimates_queue()->size()
        //        << " -- action_queue_size = "
        //        << state_estimator_.get_control_cmd_queue()->size()
        //        << ", actuation_tf_pose_estimate = ("
        //        << "loc: " << estimate_pose_local_frame_.loc.x()
        //        << "," << estimate_pose_local_frame_.loc.y()
        //        << ", angle: " <<  estimate_pose_local_frame_.angle << ")"
        //        << ", timestamp = " << timestamp
        //        << std::endl
        //        << "              queue_start_command_time " << state_estimator_.get_control_cmd_queue()->begin()->timestamp
        //        << "              queue_end_command_time " << state_estimator_.get_control_cmd_queue()->rbegin()->timestamp
        //        << std::endl;
        //PrintDbg(dbg_msg.str());

        Vector2f carrot;
        bool intersects;

        if (robot_loc_.x() > nav_params_.plan_x_end ||
                robot_loc_.x() < nav_params_.plan_x_start ||
                robot_loc_.y() < nav_params_.plan_y_start ||
                robot_loc_.y() > nav_params_.plan_y_end) {
            robot_loc_ = odom_loc_;
            cout << "ERROR: particle filter fucked up." << endl;
            throw;
        }
        if ((robot_loc_- nav_goal_loc_).norm() < nav_params_.pure_pursuit_circ_rad*2) {
            carrot = nav_goal_loc_;
            intersects = true;
            cout << "found loc goal" << endl;
        } else {
            intersects = glob_planner_.getPurePursuitCarrot(
                           robot_loc_,
                           nav_params_.pure_pursuit_circ_rad,
                           carrot);
        }
        carrot = tf::transform_point_to_loc_frame(
                PoseSE2(robot_loc_.x(),robot_loc_.y(),robot_angle_),
                carrot);

        if (!intersects) {
            // Attempt to crash recover with larger radius
            intersects = glob_planner_.getPurePursuitCarrot(
                                       robot_loc_,
                                       nav_params_.pure_pursuit_circ_rad*1.5,
                                       carrot);
            if (!intersects) {
                cout << " -------- Re-Generate-Path -------" << endl;
                PoseSE2 start(robot_loc_.x(),robot_loc_.y(),0);
                PoseSE2 goal(nav_goal_loc_.x(),nav_goal_loc_.y(),0);
                // OLEG TODO fix planner to find the nearest location that has --edges -- if there isn't a
                // big difference from the closest location
                plan_ = glob_planner_.generatePath(start, goal);
                intersects = glob_planner_.getPurePursuitCarrot(
                                       robot_loc_,
                                       nav_params_.pure_pursuit_circ_rad,
                                       carrot);
                return;
            }
            carrot = tf::transform_point_to_loc_frame(
                            PoseSE2(robot_loc_.x(),robot_loc_.y(),robot_angle_),
                            carrot);
        }

        if ((robot_loc_- nav_goal_loc_).norm() < 0.9) {
            nav_complete_ = true;
            drive_msg_.velocity = 0;
        } else {
            Vector2f local_path = PlanLocalPath(carrot);
            SetOptimalVelocity((carrot-robot_loc_).norm(), local_path(1));
        }



        drive_pub_.publish(drive_msg_);

        //printf("Latency %.2f, latency samples %ld\n", latency_tracker).estimate_latency(), latency_tracker_.get_alllatencies().size());
        //double curr_time = ros::Time::now().toSec();
        //latency_tracker_.add_controls(VelocityControlCommand{drive_msg_.velocity, curr_time});
        plot_publisher_.publish_named_point("Ctrl cmd vel", Clock::now(), drive_msg_.velocity);
        plot_publisher_.publish_named_point("Ctrl cmd c", Clock::now(), drive_msg_.curvature);
        //state_estimator_.add_control(ControlCommand(drive_msg_.velocity, drive_msg_.curvature, timestamp));


        auto w = CarDims::w;
        auto l = CarDims::l;
        visualization::DrawCar(w, l, 0x000000FF, local_viz_msg_);

        w = CarDims::w + CarDims::default_safety_margin * 2;
        l = CarDims::l + CarDims::default_safety_margin * 2;
        visualization::DrawCar(w, l, 0x0000FF00, local_viz_msg_);

        visPlan();
        visualization::DrawCross(carrot, 0.5, 0xFF0000, local_viz_msg_);

        std::vector<Vector2f> viz_pc;
        //state_estimator_.transform_p_cloud_tf_obs_to_act(
        //                           laser_pcloud_local_frame_,
        //                           viz_pc);
        //visualization::DrawPointCloud(viz_pc, 0xFF000000, local_viz_msg_);
        //PoseSE2 start(robot_loc_.x(),robot_loc_.y(),0);
        //PoseSE2 goal(nav_goal_loc_.x(),nav_goal_loc_.y(),0);

        //plan_ = glob_planner_.generatePath(start, goal);

        //Visualize path
        //for(const auto& node : plan_)
        //{
        //    Eigen::Vector2f node_loc = graph_.GetLocFromVertexIndex(node.x,node.y);
        //    std::cout << "[" << node.x << " " << node.y << "] ";
        //    visualization::DrawCross(node_loc, 0.25, 0x000FF, global_viz_msg_);
        //}
        //std::cout << std::endl;

        viz_pub_.publish(local_viz_msg_);
        viz_pub_.publish(global_viz_msg_);

        step_num_++;

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
           //     float start = fmin(ofplffset_omega, thetap);
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
