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

class DrivingControls {
    /*
	 * class that contains necessary driving functions to achieve time optimal control of robot movement.
     * controls both linear and angular movement
	 */
public:
    void update_current_speed(float distance2stop, bool initialloc_init_, float current_speed, float current_distance, float speed_increment, float c_p){
        if (current_distance >= Assignment1::target_dis or !initialloc_init_){
            new_velocity = 0;
        } else if (current_distance + distance2stop >= Assignment1::target_dis) {
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