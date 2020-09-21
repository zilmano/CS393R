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
    a = R + w / 2.f;
    b = (l + CarDims::wheelbase) / 2.f;
    corner_params(0, 0) = sqrtf(a * a + b * b);
    corner_params(0, 1) = atanf((l + CarDims::wheelbase) / (2 * R + w)) - M_PI /2.f;
    a = R - w / 2.f;
    b = (l + CarDims::wheelbase) / 2.f;
    corner_params(1, 0) = sqrtf(a * a + b * b);
    corner_params(1, 1) = atanf((l + CarDims::wheelbase) / (2 * R - w)) - M_PI /2.f;
    a = R - w / 2.f;
    b = (l - CarDims::wheelbase) / 2.f;
    corner_params(2, 0) = sqrtf(a * a + b * b);
    corner_params(2, 1) = atanf(-(l - CarDims::wheelbase) / (2 * R - w)) - M_PI /2.f;
    a = R + w / 2.f;
    b = (l - CarDims::wheelbase) / 2.f;
    corner_params(3, 0) = sqrtf(a * a + b * b);
    corner_params(3, 1) = atanf(-(l - CarDims::wheelbase) / (2 * R + w)) - M_PI /2.f;
    return corner_params;
}

xt::xtensor<float, 1> CollisionPlanner::convert_pts2cspace(const xt::xtensor<float, 1> &pt, float curvature) {
    float R = 1.f / curvature;
    xt::xtensor<float, 1> pt_params({2});
    xt::xtensor<float, 1> center{0, R};
    pt_params[0] = xt::eval(xt::sqrt(xt::sum(xt::square(pt - center))))[0];
    pt_params[1] = atanf(pt[0] / (R - pt[1])) - M_PI /2.f;
    return pt_params;
}

float CollisionPlanner::linear_interpolate_params_wrt_R(const xt::xtensor<float, 1> &pt1,
                                                        const xt::xtensor<float, 1> &pt2, float Rp) {
    return (abs(Rp - pt1[0]) * pt1[1] + abs(Rp - pt2[0]) * pt2[1]) / abs(pt1[0] - pt2[0]);
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
        if ((pt_param[0] >= min_R && pt_param[0] <= max_R)[0]) potential_collisions.emplace_back(xt_pt);
    }
    return potential_collisions;
}