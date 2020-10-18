//
// Created by liyanc on 10/11/20.
//

#ifndef REPO_MOTION_MODEL_H
#define REPO_MOTION_MODEL_H

#include "latencytracking.hpp"
#include "particle_filter.h"
#include "shared/util/random.h"
#include <vector>


using particle_filter::Particle;
using util_random::Random;

class MotionModel {

public:
    MotionModel(float sigma_cur, float sigma_vel);

    void register_command(float curvature, float vel);

    void set_params(float sigma_c, float sigma_v);

    std::vector<Particle> &transform_pose(std::vector<Particle> &particles, Random &rng, Eigen::Vector2f &dis, float d_theta);

private:
    float calculate_delta_t();

    float sigma_cur, sigma_vel;

    float last_cmd_cur, last_cmd_vel, last_cmd_time;
};



#endif //REPO_MOTION_MODEL_H
