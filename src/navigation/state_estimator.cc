/*
 * state_estimator.cc
 *
 *  Created on: Sep 19, 2020
 *      Author: olegzilm
 */
#ifndef NAV_STATE_ESTIM_H
#define NAV_STATE_ESTIM_H

#define _USE_MATH_DEFINES

#include "state_estimator.h"
#include "shared/global_utils.h"
#include "shared/constants.h"
#include <cmath>
#include <limits>

// Oleg TODO: remove.
#include <iostream>



float StateEstimator::calc_arc_path(float theta, float curvature) {
     if (fabs(curvature) <= GenConsts::kEpsilon) {
         return std::numeric_limits<float>::infinity();
     }

     if (theta > M_PI) {
         theta = 2*M_PI - theta;
     }
     if (fabs(theta) <= GenConsts::kEpsilon) {
        return 0.0f;
     }
     float r = fabs(1/curvature);
     return r * sqrt(2*(1-cos(theta)));
}

PoseSE2 StateEstimator::pose_change_one_step_forward(float vel , float curvature,
                                                     float period_ratio) {
    if (fabs(vel) < GenConsts::kEpsilon) {
        return PoseSE2();
    }

    if (fabs(curvature) <= GenConsts::kEpsilon) {
        return PoseSE2(vel*GenConsts::step_period*period_ratio,0,0);
    }

    float ang_vel = fabs(vel)*curvature;
    float theta = ang_vel*GenConsts::step_period*period_ratio;
    //std::cout << " state_estimator::theta: " << theta;
    float arc_path = calc_arc_path(theta, curvature);
    float delta_angle = theta;
    if (vel < 0) {
        theta = 2*M_PI - theta;
    }
    float arc_path_angle = theta/2;
    float delta_x = cos(arc_path_angle)*arc_path;
    float delta_y = sin(arc_path_angle)*arc_path;
    PoseSE2 delta_pose(delta_x, delta_y, delta_angle);
    return delta_pose;
}

void StateEstimator::transform_p_cloud_tf_obs_to_act(
        const std::vector<Vector2f>& point_cloud,
        std::vector<Vector2f>& point_cloud_actuation_tf) {
    // Convert point cloud from observation time frame to actuation time frame
    point_cloud_actuation_tf.clear();
    Vector2f actuation_tf_point;
    for (size_t i = 0; i < point_cloud.size(); ++i) {
        if (point_cloud[i].norm() <
            PhysicsConsts::radar_max_range-2*PhysicsConsts::radar_noise_std) {
            actuation_tf_point = tf::transform_point_to_loc_frame(
                    actuation_time_pose_estimate_,
                    point_cloud[i]);
        } else {
            actuation_tf_point = point_cloud[i];
        }
        point_cloud_actuation_tf.push_back(actuation_tf_point);
    }
}

void StateEstimator::update_estimation(const Vector2f& observation_loc, float observation_angle,
                                       double curr_time) {
    curr_time_ = curr_time;
    float observation_time = curr_time_ - observation_latency_;

    pose_estimates_.clear();
    long unsigned pe_index = 1;
    PoseSE2 observation_pose(observation_loc,observation_angle);
    pose_estimates_.push_back(observation_pose);
    PoseSE2 accumulated_pose = observation_pose;

    double actuation_step_period_ratio =
        (actuation_latency_ / GenConsts::step_period);
    double actuation_to_step_period_skew =
        (actuation_step_period_ratio - floor(actuation_step_period_ratio));
   
    // OLEG TODO: remove this is for debug
    /*auto print_pose =  [](const PoseSE2& p) {
              std::cout << " x:" << p.loc.x() << " y:" << p.loc.y() << " angle:" << p.angle;
    };*/
    std::cout << std::endl << "-- update_estimation::" << std::endl
              << "    actuation_latency " << actuation_latency_
              << "    obs_latency " << observation_latency_
              << "    obs_time " << observation_time << std::endl;


    for (std::list<ControlCommand>::const_iterator cmd_it = cmd_quasi_queue_.begin();
        cmd_it != cmd_quasi_queue_.end();) {
        double cmd_actuation_time = cmd_it->timestamp + actuation_latency_;
        if (cmd_actuation_time < observation_time) {
            cmd_it = cmd_quasi_queue_.erase(cmd_it);
        } else {
            if (cmd_actuation_time < curr_time) {
                pose_est_index_of_curr_step_ = pe_index;
            }
            PoseSE2 delta_pose = pose_change_one_step_forward(
                cmd_it->vel,
                cmd_it->c);
            PoseSE2 delta_pose_part = pose_change_one_step_forward(
                cmd_it->vel,
                cmd_it->c,
                1-actuation_to_step_period_skew);

            pose_estimates_.push_back(
                    accumulated_pose +
                    tf::transform_pose_to_glob_frame(
                        observation_pose,
                        delta_pose_part)
            );


            accumulated_pose += tf::transform_pose_to_glob_frame(
                    observation_pose,
                    delta_pose);

            ++pe_index;
            ++cmd_it;
        }
    }
    actuation_time_pose_estimate_ = pose_estimates_[pose_estimates_.size()-1];
}

#endif // NAV_STATE_ESTIM_H

