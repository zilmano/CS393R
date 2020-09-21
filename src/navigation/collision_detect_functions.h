/*
 * collision_detect
 *  Created on: Sep 21, 2020
 *      Author: olegzilm
 */
#define _USE_MATH_DEFINES

#include "eigen3/Eigen/Dense"
#include "constants.h"
#include <cmath>

using Eigen::Vector2f;

namespace collision {

    bool is_point_in_circle(Vector2f circle_center, float radius, Vector2f point) {
        if ((circle_center-point).norm() <= radius) {
            return true;
        }
        return false;
    }

    bool is_point_in_path(float r, float clearance, const Vector2f& point) {
        Vector2f turning_center(0,r);
        float collision_ring_internal_r = r-CarDims::w/2-clearance;
        float collision_ring_external_r = r+CarDims::w/2+clearance;
        if (!is_point_in_circle(turning_center,collision_ring_internal_r,point) &&
             is_point_in_circle(turning_center,collision_ring_external_r,point)) {
            return true;
        }
        return false;
    }

    float calc_distance_on_curve_from_angle_to_point(float angle, float c) {
        float arc = angle*2;
        if (fabs(c) < GenConsts::kEpsilon) {
            return std::numeric_limits<float>::infinity();
        }
        float r = 1/c;
        float dist = r*arc;
        return dist;

    }

    float calc_distance_on_curve_to_point(float c, const Vector2f& point) {
        if (fabs(c) < GenConsts::kEpsilon) {
           return point.norm();
        }
        float angle = atan(point.y()/point.x());

        return calc_distance_on_curve_from_angle_to_point(angle, c);

    }


    float check_collision_curvature_zero(std::vector<Vector2f> point_cloud) {
        float upper_y_bound = CarDims::w/2 + CarDims::default_safety_margin;
        float lower_y_bound = -CarDims::w/2 - CarDims::default_safety_margin;
        float fpl = GenConsts::FPL_max_bound;
        for (size_t i = 0; i < point_cloud.size(); ++i) {
            Vector2f curr_p = point_cloud[i];
            if (curr_p.y() < upper_y_bound && curr_p.y() > lower_y_bound) {
                fpl = curr_p.x() - CarDims::wheelbase - CarDims::default_safety_margin;
            }
        }
        return fpl;
    }



}


