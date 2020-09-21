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
    void update_current_speed(float distance2stop, bool initialloc_init_, float current_speed, float current_distance, float speed_increment, float c_p, float target){
        if (current_distance <= 0 or !initialloc_init_){
            new_velocity = 0;
        } else if (current_distance <= 0.18) {
            current_speed -= speed_increment + current_distance * c_p;
            new_velocity = current_speed;
        } else if (current_speed >= PhysicsConsts::max_vel) {
            current_speed = PhysicsConsts::max_vel;
            new_velocity = current_speed;
        } else {
            current_speed += speed_increment;
            new_velocity = current_speed;
        }
    }
    
    // distance until end of fpl
    float calculate_current_distance(float curvature, Vector2f robot_location, Vector2f target, float arc_length){
        float current_distance;
        if (curvature >= 0){
            current_distance = 1 / curvature * calculate_theta(robot_location, target, curvature);
        } else {
            current_distance = (-1) * 1 / curvature * calculate_theta(robot_location, target, curvature);
        }
        
        return current_distance;
    }
    
    // calculates cartesian coordinates of target location (end of fpl)
    Vector2f calculate_target_location(float arc_length, float curvature, float init_angle, Vector2f center){
        float theta = arc_length * curvature;
        float x;
        float y;
        if (curvature > 10000){
            curvature = 10000;
        }
        if (curvature < 0){
            theta = 2 * M_PI - theta;
            x = center.x() + 1/curvature * cos(init_angle - theta);
            y = center.y() + 1/curvature * sin(init_angle - theta);
        }
        else {
            x = center.x() + 1/curvature * cos(theta + init_angle);
            y = center.y() + 1/curvature * sin(theta + init_angle);
        }
       
        Vector2f target(x,y);

        return target;
    }
    
    // angle between target and robot location when new path is initialized
    float calculate_initial_angle(float robot_angle, float curvature){
        float theta;
        if (curvature < 0) {
            theta = robot_angle + M_PI/2;
        } else {
            theta = robot_angle - M_PI/2;
        }

        return theta;
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
    
    Vector2f get_center_of_turning(float curvature, Vector2f robot_location, float robot_angle){
        if (curvature > 10000){
            curvature = 10000;
        }
        float theta;
        if (curvature < 0){
            theta = robot_angle - M_PI/2;
        } else {
            theta = robot_angle + M_PI/2;
        }

        float x_center = robot_location.x() + 1/curvature * cos(theta);
        float y_center = robot_location.y() + 1/curvature * sin(theta);
        Vector2f center(x_center,y_center);
        return center;
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

    // angle between two points based on center of turning
    float calculate_theta(Vector2f starting_point, Vector2f ending_point, float curvature){
        float dist_between_points = (ending_point - starting_point).norm();
        float theta;

        if(curvature >= 0){
            theta = 2 * asin(0.5 * dist_between_points * curvature);
            //theta = atan(ending_point.y()/ending_point.x()) - atan(starting_point.y()/starting_point.x());
        }
        else {
            theta = (-1) * 2 * asin(0.5 * dist_between_points * curvature);
            //theta = -(atan(ending_point.y()/ending_point.x()) - atan(starting_point.y()/starting_point.x()));
        }

        return theta;
    }

private:
    float new_velocity;
    float new_angular_velocity;



};




#endif /* SRC_NAVIGATION_WORLD_DRIVINGCONTROLS_H_ */