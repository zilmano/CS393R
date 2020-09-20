//
// Created by liyanc on 9/20/20.
//

#ifndef REPO_COLLISIONPLANNER_H
#define REPO_COLLISIONPLANNER_H

#include "world.h"
#include "xtensor/xtensor.hpp"

class CollisionPlanner {
public:
    CollisionPlanner(World &pt_cloud);

    xt::xtensor<float, 2> convert4corner2cspace(float curvature);

    xt::xtensor<float, 1> convert_pts2cspace(const xt::xtensor<float, 1> &pt, float curvature);

    float linear_interpolate_params_wrt_R(const xt::xtensor<float, 1> &pt1, const xt::xtensor<float, 1> &pt2, float Rp);



private:
    World &point_cloud_;
};


#endif //REPO_COLLISIONPLANNER_H
