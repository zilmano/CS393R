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

using Eigen::Vector2f;


struct PoseSE2 {
        PoseSE2(): loc{0,0}, angle{0} {};
        PoseSE2(float x,float y,float angle_init):
                loc{x,y}, angle{angle_init} {};
        PoseSE2(Vector2f loc, float angle_init):
                loc{loc}, angle{angle_init} {};

        Vector2f loc;
        float angle;

        const PoseSE2 operator*(double ratio) {
            return PoseSE2(ratio*(this->loc),ratio*(this->angle));
        }

        const PoseSE2 operator+(const PoseSE2& rhs) {
            PoseSE2 result;
            result.loc = this->loc + rhs.loc;
            result.angle = this->angle + rhs.angle;
            return result;
        }

        PoseSE2& operator+=(const PoseSE2& rhs) {
            this->loc += rhs.loc;
            this->angle += rhs.angle;
            return *this;
        }
};

/*const PoseSE2 operator*(double ratio, const PoseSE2& rhs ) {
            return PoseSE2(ratio*(rhs.loc),ratio*(rhs.angle));
}*/

// Oleg TODO: consolidate with latencytracker class.
struct OdomMeasurement {
    float vel;
    float c;
    Eigen::Vector2f loc;
    float angle;
    double timestamp;
};

struct ControlCommand {
    ControlCommand(float vel_init,float c_init,double timestamp_init):
                   vel(vel_init),c(c_init),timestamp(timestamp_init) {};
    float vel;
    float c;
    double timestamp;
};


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

    void set_latency(float observation_latency,float actuation_latency) {
        observation_latency_ = observation_latency;
        actuation_latency_ = actuation_latency;
    }

    void add_control(const ControlCommand& curr_command) {
        cmd_quasi_queue_.push_back(curr_command);
    }

    PoseSE2 estimate_state_curr_time(double curr_time) {
        if (curr_time != curr_time_) {
            throw "StateEstimator:: StateEstimator needs to be updated with 'update_estimation()' prior ti running 'estimate_state_for_curr_time()";
        }
        return pose_estimates_[pose_est_index_of_curr_step_];
    };
    PoseSE2 estimate_state_cmd_actuation_time(double curr_time) {
        if (curr_time != curr_time_) {
            throw "StateEstimator:: StateEstimator needs to be updated with 'update_estimation()' prior ti running 'estimate_state_for_curr_time()";
        }
        return pose_estimates_[pose_estimates_.size()-1];
    };



private:
    float calc_arc_path(float theta, float curvature);
    PoseSE2 pose_change_one_step_forward(float vel, float curvature);

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


};







#endif /* SRC_NAVIGATION_STATE_ESTIMATOR_H_ */
