/*
 * drivingcontrols.h
 *
 *  Created on: Sep 19, 2020
 *      Author: Nathan Nguyen
 */

#ifndef SRC_NAVIGATION_DRIVINGCONTROLS_H_
#define SRC_NAVIGATION_DRIVINGCONTROLS_H_


#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include <vector>
#include <memory>
#include <math.h> 
#include <cmath>

using Eigen::Vector2f;
using Eigen::Matrix2f;
using Eigen::Rotation2Df;

class DrivingControls {
    /*
	 * class that contains necessary driving functions to achieve time optimal control of robot movement.
     * controls both linear and angular movement
	 */
public:
    // TOC
    void update_current_speed(float dis2stop, float spd_inc, bool initial_loc_init, float curr_spd, float c_p){
        float dis2target = _target_dist - _dist_traveled;        
        //std::cout << "dis2targ: " << dis2target << std::endl;
        //std::cout << "dis2stop: " << dis2stop << std::endl;
        //std::cout << "dis_traveled: " << _dist_traveled << std::endl;
        if (dis2target <= 0 or !initial_loc_init){
            _new_velocity = 0;
        } else if (dis2stop >= dis2target) {
            curr_spd -= spd_inc + dis2target * c_p;
            _new_velocity = curr_spd;
        } else if (curr_spd >= PhysicsConsts::max_vel) {
            curr_spd = PhysicsConsts::max_vel;
            _new_velocity = curr_spd;
        } else {
            curr_spd += spd_inc;
            _new_velocity = curr_spd;
        }
    }

    float update_dist_traveled(float curr_spd, float actuation_latency, float target_dist, float curvature){
        if(target_dist != _target_dist or curvature != _curvature){
            _target_dist = target_dist;
            _curvature = curvature;
            _dist_traveled = 0;
        }
        _dist_traveled += curr_spd * actuation_latency;

        return _dist_traveled;
    }

    float get_new_velocity(){
        return _new_velocity;
    }

        // double check that velocity isn't exceeding max velocity setting
    float drive_msg_check(float drv_msg){
        if (drv_msg > 1.0){
            drv_msg = 1;
        } else if (drv_msg < 0.0){
            drv_msg = 0;
        } else {
            drv_msg = drv_msg;
        }
        return drv_msg;
    }

private:
    float _dist_traveled;
    float _new_velocity;
    float _target_dist;
    float _curvature;

};




#endif /* SRC_NAVIGATION_WORLD_DRIVINGCONTROLS_H_ */