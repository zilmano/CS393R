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

using Eigen::Vector2f;
using Eigen::Matrix2f;
using Eigen::Rotation2Df;

class DrivingControls {
    /*
	 * class that contains necessary driving functions to achieve time optimal control of robot movement.
     * controls both linear and angular movement
	 */
public:
    void update_current_speed(float distance2stop, bool initialloc_init_, float current_speed, float current_distance, float speed_increment, float c_p){
        if (current_distance >= (current_distance + Assignment1::target_dis) or !initialloc_init_){
            new_velocity = 0;
        } else if (current_distance + distance2stop >= (current_distance + Assignment1::target_dis)) {
            current_speed -= speed_increment + (Assignment1::target_dis - current_distance) * c_p;
            new_velocity = current_speed;
        } else if (current_speed >= PhysicsConsts::max_vel) {
            current_speed = PhysicsConsts::max_vel;
            new_velocity = current_speed;
        } else {
            current_speed += speed_increment;
            new_velocity = current_speed;
        }
    }
    
    Vector2f calculate_target_location(Vector2f robot_location, float robot_angle){
        robot_location.x() = robot_location.x() + Assignment1::target_dis * cos(robot_angle);
        robot_location.y() = robot_location.y() + Assignment1::target_dis * sin(robot_angle);

        return robot_location;
    }

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
    

    void update_current_angular_velocity(){
        // needs to take into account targeted direction
    }

    float get_velocity(){
        return new_velocity;
    }

    float get_angular_velocity(){
        return new_angular_velocity;
    }

private:
    float new_velocity;
    float new_angular_velocity;


};




#endif /* SRC_NAVIGATION_WORLD_DRIVINGCONTROLS_H_ */