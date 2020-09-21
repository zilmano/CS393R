//
// Created by liyanc on 9/20/20.
//

#include <vector>
#include <algorithm>
#include "collisionplanner.h"
#include "constants.h"
#include "cmath"
#include "xtensor/xmath.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor-blas/xlinalg.hpp"

CollisionPlanner::CollisionPlanner(World &pt_cloud) :
        point_cloud_{pt_cloud} {}


xt::xtensor<float, 2> CollisionPlanner::convert4corner2cspace(float curvature) {
    // corner_params[:, 0] is the radius of the corner wrt. the turning center
    // corner_params[:, 1] is the angle of the corner wrt. the base_link and the turning center
    xt::xtensor<float, 2> corner_params({4, 2});
    float w = CarDims::w + CarDims::default_safety_margin * 2;
    float l = CarDims::l + CarDims::default_safety_margin * 2;
    float R = 1.0f / curvature;
    float a, b;
    float offset_omega = curvature > 0 ? (-M_PI / 2.f) : (M_PI / 2.f);
    a = R + w / 2.f;
    b = (l + CarDims::wheelbase) / 2.f;
    corner_params(0, 0) = sqrtf(a * a + b * b);
    corner_params(0, 1) = atanf((l + CarDims::wheelbase) / (2 * R + w)) + offset_omega;
    a = R - w / 2.f;
    b = (l + CarDims::wheelbase) / 2.f;
    corner_params(1, 0) = sqrtf(a * a + b * b);
    corner_params(1, 1) = atanf((l + CarDims::wheelbase) / (2 * R - w)) + offset_omega;
    a = R - w / 2.f;
    b = (l - CarDims::wheelbase) / 2.f;
    corner_params(2, 0) = sqrtf(a * a + b * b);
    corner_params(2, 1) = atanf(-(l - CarDims::wheelbase) / (2 * R - w)) + offset_omega;
    a = R + w / 2.f;
    b = (l - CarDims::wheelbase) / 2.f;
    corner_params(3, 0) = sqrtf(a * a + b * b);
    corner_params(3, 1) = atanf(-(l - CarDims::wheelbase) / (2 * R + w)) + offset_omega;
    return corner_params;
}

xt::xtensor<float, 1> CollisionPlanner::convert_pts2cspace(const xt::xtensor<float, 1> &pt, float curvature) {
    float R = 1.f / curvature;
    float offset_omega = curvature > 0 ? (-M_PI / 2.f) : (M_PI / 2.f);
    xt::xtensor<float, 1> pt_params{0.0, 0.0};
    xt::xtensor<float, 1> center{0, R};
    pt_params[0] = xt::linalg::norm(pt - center);
    pt_params[1] = atanf(pt[0] / (R - pt[1])) + offset_omega;
    return pt_params;
}

float CollisionPlanner::linear_interpolate_params_wrt_R(const xt::xtensor<float, 1> &pt1,
                                                        const xt::xtensor<float, 1> &pt2, float Rp) {
    return (abs(Rp - pt1[0]) * pt2[1] + abs(Rp - pt2[0]) * pt1[1]) / abs(pt1[0] - pt2[0]);
}

std::vector<xt::xtensor<float, 1>>
CollisionPlanner::select_potential_collision(float curvature, std::vector<Eigen::Vector2f> &pts) {
    //float R = 1.f / curvature;
    std::vector<xt::xtensor<float, 1>> potential_collisions;
    potential_collisions.reserve(pts.size());
    auto corner_params = convert4corner2cspace(curvature);
    auto min_R = xt::amin(xt::col(corner_params, 0));
    auto max_R = xt::amax(xt::col(corner_params, 0));
    for (auto &pt : pts) {
        xt::xtensor<float, 1> xt_pt = xt::adapt(pt.data(), {2});
        auto pt_param = convert_pts2cspace(xt_pt, curvature);
        if ((pt_param[0] >= min_R && pt_param[0] <= max_R)[0]) potential_collisions.emplace_back(pt_param);
    }
    return potential_collisions;
}

float CollisionPlanner::calculate_shortest_collision(float curvature, std::vector<xt::xtensor<float, 1>> & colliding_pts) {
    auto corner_params = convert4corner2cspace(curvature);
    std::vector<float> offending_angle{};
    offending_angle.reserve(3 * colliding_pts.size());

    float top_inner_R = fminf(corner_params(0, 0), corner_params(1, 0));
    float top_outter_R = fmaxf(corner_params(0, 0), corner_params(1, 0));
    float left_inner_R = fminf(corner_params(1, 0), corner_params(2, 0));
    float left_outter_R = fmaxf(corner_params(1, 0), corner_params(2, 0));
    float right_inner_R = fminf(corner_params(0, 0), corner_params(3, 0));
    float right_outter_R = fmaxf(corner_params(0, 0), corner_params(3, 0));
    for (auto & pt: colliding_pts) {
        // Top side
        if (pt[0] > top_inner_R && pt[0] < top_outter_R) {
            float front_theta = linear_interpolate_params_wrt_R(
                    xt::row(corner_params, 0), xt::row(corner_params, 1), pt[0]);
            float diff = pt[1] - front_theta;
            offending_angle.emplace_back(diff);
        }
        // Left side
        if (pt[0] > left_inner_R && pt[0] < left_outter_R) {
            float front_theta = linear_interpolate_params_wrt_R(
                    xt::row(corner_params, 1), xt::row(corner_params, 2), pt[0]);
            float diff = pt[1] - front_theta;
            offending_angle.emplace_back(diff);
        }
        // Right side
        if (pt[0] > right_inner_R && pt[0] < right_outter_R) {
            float front_theta = linear_interpolate_params_wrt_R(
                    xt::row(corner_params, 3), xt::row(corner_params, 0), pt[0]);
            float diff = pt[1] - front_theta;
            offending_angle.emplace_back(diff);
        }
    }
    if (curvature > 0){
        if (!offending_angle.empty()) return *std::min(offending_angle.begin(), offending_angle.end());
        else return INFINITY;
    }
    else {
        if (!offending_angle.empty()) return *std::max(offending_angle.begin(), offending_angle.end());
        else return INFINITY;
    }

}

std::vector<float> CollisionPlanner::generate_candidate_paths(float c_step, float min_radius) {
        std::vector<float> candidates;
        candidates.push_back(0);
        c_step = fabs(c_step);
        if (fabs(c_step) < GenConsts::kEpsilon) {
            throw "::generate_candidate_paths: c_step must be bigger then zero (by kEpsilon)";
        }
        float c = c_step;
        float r = 1/c;
        while (r >= min_radius) {
            candidates.push_back(-1*c);
            candidates.push_back(c);
            c += c_step;
            r = 1/c;
        }
        return candidates;
} 

float CollisionPlanner::calc_dist_on_curve_for_angle(float curvature, float angle) {
    // OLEG TODO:: to impelemnt!
    return angle;
}

float CollisionPlanner::calculate_shortest_collision_flat(std::vector<Eigen::Vector2f> & pts) {
    std::vector<float> offending_distances;
    float w = CarDims::w + CarDims::default_safety_margin * 2;
    offending_distances.reserve(pts.size());
    for (auto & pt: pts)
        if (pt[0] > 0 && fabsf(pt[1]) < w / 2.f)
            offending_distances.emplace_back(pt[0]);
    if (!offending_distances.empty()) return *std::min(offending_distances.begin(), offending_distances.end());
    else return INFINITY;
}

float CollisionPlanner::calculate_shortest_translational_displacement(
        float curvature, std::vector<Eigen::Vector2f> & pts) {
    if (fabsf(curvature) < GenConsts::kEpsilon)
        return calculate_shortest_collision_flat(pts);
    else {
        float R = 1.f / curvature;
        auto collisions = select_potential_collision(curvature, pts);
        float theta = calculate_shortest_collision(curvature, collisions);
        return R * theta;
    }
}