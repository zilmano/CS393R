//
// Created by liyanc on 9/20/20.
//

#include "collisionplanner.h"
#include "constants.h"
#include "cmath"
#include "xtensor/xmath.hpp"
#include "xtensor-blas/xlinalg.hpp"

CollisionPlanner::CollisionPlanner(World &pt_cloud) :
    point_cloud_{pt_cloud}
    {}


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
    corner_params(0, 1) = atanf((l + CarDims::wheelbase) / (2 * R + w));
    a = R - w / 2.f;
    b = (l + CarDims::wheelbase) / 2.f;
    corner_params(1, 0) = sqrtf(a * a + b * b);
    corner_params(1, 1) = atanf((l + CarDims::wheelbase) / ( 2 * R - w));
    a = R - w / 2.f;
    b = (l - CarDims::wheelbase) / 2.f;
    corner_params(2, 0) = sqrtf(a * a + b * b);
    corner_params(2, 1) = atanf((l - CarDims::wheelbase) / ( 2 * R - w));
    a = R + w / 2.f;
    b = (l - CarDims::wheelbase) / 2.f;
    corner_params(3, 0) = sqrtf(a * a + b * b);
    corner_params(3, 1) = atanf((l - CarDims::wheelbase) / (2 * R + w));
    return corner_params;
}

xt::xtensor<float, 1> CollisionPlanner::convert_pts2cspace(const xt::xtensor<float, 1> &pt, float curvature) {
    float R = 1.f / curvature;
    xt::xtensor<float, 1> pt_params({2});
    xt::xtensor<float, 1> center{R, 0};
    pt_params[0] = xt::linalg::norm(pt - center, 2);
    pt_params[1] = atanf(pt[1] / (pt[0] + R));
    return pt_params;
}

float CollisionPlanner::linear_interpolate_params_wrt_R(const xt::xtensor<float, 1> &pt1,
                                                        const xt::xtensor<float, 1> &pt2, float Rp) {
    return (abs(Rp - pt1[0]) * pt1[1] + abs(Rp - pt2[0]) * pt2[1]) / abs(pt1[0] - pt2[0]);
}