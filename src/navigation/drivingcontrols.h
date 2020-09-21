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
    void update_current_speed(float distance2stop, bool initialloc_init_, float current_speed, float current_distance, float speed_increment, float c_p){
        if (current_distance <= 0 or !initialloc_init_){
            new_velocity = 0;
        } else if (current_distance <= distance2stop) {
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
    
    float calculate_current_distance(float curvature, Vector2f robot_location, Vector2f target, float arc_length){
        return arc_length - 1 / curvature * calculate_theta(robot_location, target, curvature);
    }

    
    Vector2f calculate_target_location(float arc_length, float curvature, float init_angle, Vector2f robot_location){
        float theta = arc_length * curvature;
        if (curvature > 10000){
            curvature = 10000;
        }
        robot_location.x() = robot_location.x() + 1/curvature * cos(theta + init_angle);
        robot_location.y() = robot_location.y() + 1/curvature * sin(theta + init_angle);

        return robot_location;
    }
    
    float calculate_initital_angle(Vector2f robot_location, Vector2f center){
        float x_rel = robot_location.x() - center.x();
        float y_rel = robot_location.y() - center.y();
        float theta = atan(y_rel/x_rel);
        if (theta > M_PI) {
        theta = 2*M_PI - theta;
        }
        return theta;
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
    
    Vector2f get_center_of_turning(float curvature, Vector2f robot_location, float robot_angle, Vector2f init_loc){
        if (curvature > 10000){
            curvature = 10000;
        }
        float x_center = init_loc.x() + 1/curvature * cos(robot_angle);
        float y_center = init_loc.y() + 1/curvature * sin(robot_angle);
        Vector2f center(x_center,y_center);
        return center;
    }

    void update_current_angular_velocity(){
        // needs to take into account targeted direction
    }

    float get_velocity(){
        return new_velocity;
    }

    float get_angular_velocity(float curvature){
        return new_velocity * curvature;
    }

    Vector2f get_polar_coordinates(Vector2f robot_location){
        float radius = robot_location.norm();
        float angle = atan(robot_location.y()/robot_location.x());
        Vector2f polar_coord(radius, angle);
        return polar_coord;
    }

    float calculate_theta(Vector2f starting_point, Vector2f ending_point, float curvature){
        float dist_between_points = (ending_point - starting_point).norm();
        float theta = 2 * asin(0.5 * dist_between_points * curvature);
        if (theta > M_PI) {
            theta = 2*M_PI - theta;
        }
        return theta;
    }

private:
    float new_velocity;
    float new_angular_velocity;



};




#endif /* SRC_NAVIGATION_WORLD_DRIVINGCONTROLS_H_ */