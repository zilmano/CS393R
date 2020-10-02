/*
 * state_estimator.h
 *
 *  Created on: Sep 19, 2020
 *      Author: olegzilm
 */

#ifndef SRC_NAVIGATION_STATE_ESTIMATOR_H_
#define SRC_NAVIGATION_STATE_ESTIMATOR_H_

#include <list>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "collision_detect_functions.h"

using Eigen::Vector2f;
using navigation::PoseSE2;
using navigation::ControlCommand;


class StateEstimator {

public:
    explicit StateEstimator(double actuation_latency, double observation_latency, double curr_time):
          actuation_latency_{actuation_latency},
          observation_latency_{observation_latency},
          curr_time_{curr_time},
          pose_est_index_of_curr_step_{0}
          {};

    /*void update_estimation(const Vector2f observation_loc, float observation_pose,
                           unsigned long step_num) {

    };*/

    void update_estimation(const Vector2f& observation_loc, float observation_angle,
                           double curr_time);

    void transform_p_cloud_tf_obs_to_act(
            const std::vector<Vector2f>& point_cloud,
            std::vector<Vector2f>& point_cloud_actuation_tf);

    inline void set_latency(float observation_latency,float actuation_latency) {
        observation_latency_ = observation_latency;
        actuation_latency_ = actuation_latency;
    }

    inline void add_control(const ControlCommand& curr_command) {
        cmd_quasi_queue_.push_back(curr_command);
    }

    inline PoseSE2 estimate_state_curr_time(double curr_time) {
        if (curr_time != curr_time_) {
            throw "StateEstimator:: StateEstimator needs to be updated with 'update_estimation()' prior ti running 'estimate_state_for_curr_time()";
        }
        return pose_estimates_[pose_est_index_of_curr_step_];
    };

    inline PoseSE2 estimate_state_cmd_actuation_time(double curr_time) {
        if (curr_time != curr_time_) {
            throw "StateEstimator:: StateEstimator needs to be updated with 'update_estimation()' prior ti running 'estimate_state_for_curr_time()";
        }
        return actuation_time_pose_estimate_;
    };

    inline std::vector<PoseSE2>* get_pose_estimates_queue() {
        return &pose_estimates_;
    }

    inline std::list<ControlCommand>* get_control_cmd_queue() {
        return &cmd_quasi_queue_;
    }



private:
    float calc_arc_path(float theta, float curvature);
    PoseSE2 pose_change_one_step_forward(float vel, float curvature,float period_ratio=1);

private:
    double actuation_latency_;
    double observation_latency_;
    double curr_time_;


    // index of curr_time step in pose_estimation vector
    unsigned long pose_est_index_of_curr_step_;
    // Quasi queue to implement the latency compensation
    std::list<ControlCommand> cmd_quasi_queue_;

    // pose history for time steps until the latest known odometry.
    std::vector<PoseSE2> pose_history_;

    // Estimated poses since that latest known odometry
    std::vector<PoseSE2> pose_estimates_;
    PoseSE2 actuation_time_pose_estimate_;


};







#endif /* SRC_NAVIGATION_STATE_ESTIMATOR_H_ */
