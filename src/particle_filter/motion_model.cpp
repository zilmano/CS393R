//
// Created by liyanc on 10/11/20.
//

#include "clock.h"
#include "constants.h"
#include "motion_model.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/constants.h"

MotionModel::MotionModel(float sigma_cur, float sigma_vel) :
        sigma_cur(sigma_cur),
        sigma_vel(sigma_vel),
        last_cmd_time(-1.0) {}

inline Eigen::Vector2f find_rotation_center(Eigen::Vector2f &loc, float theta, float radius) {
    auto rotation = Eigen::Rotation2D<float>(theta + M_PI / 2.f);
    Eigen::Vector2f rad{radius, 0};
    auto center = rotation * rad;
    return center;
}

inline Eigen::Vector2f
rotate_loc_wrt_center_and_angle(Eigen::Vector2f &loc, Eigen::Vector2f &center, float delta_theta) {
    return Eigen::Rotation2D<float>(delta_theta) * (loc - center) + center;
}

void MotionModel::register_command(float curvature, float vel) {
    last_cmd_cur = curvature;
    last_cmd_vel = vel;
    last_cmd_time = Clock::now();
}

float MotionModel::calculate_delta_t() {
    return fabs(Clock::now() - last_cmd_time);
}

void MotionModel::set_params(float sigma_c, float sigma_v) {
    sigma_cur = sigma_c;
    sigma_vel = sigma_v;
}

std::vector<Particle> & MotionModel::transform_pose(std::vector<Particle> &particles, Random &rng, Eigen::Vector2f &dis,
                                                    float d_theta) {
    if (last_cmd_time < 0.0) return particles;
    else {
        for (auto & p: particles) {
            float epsilon_cur = rng.Gaussian(0, sigma_cur);
            float epsilon_vel = rng.Gaussian(0, sigma_vel);
            float curvature = last_cmd_cur + epsilon_cur;
            //float delta_t = calculate_delta_t();
            float delta_t = dis.norm() / (epsilon_vel + last_cmd_vel);
            float delta_theta = delta_t * (epsilon_vel + last_cmd_vel) * curvature;
            float radius = 1.f / curvature;
            float delta_d = epsilon_vel * delta_t;
            Eigen::Vector2f d_loc(delta_d * cos(p.angle), delta_d * sin(p.angle));
            auto center = find_rotation_center(p.loc, p.angle, radius);
            if (fabs(curvature) < GenConsts::kEpsilon) p.loc += dis + d_loc;
            else p.loc = (rotate_loc_wrt_center_and_angle(p.loc, center, delta_theta) + p.loc + dis) / 2;
            p.angle += (delta_theta + d_theta ) / 2;
        }
        return particles;
    }
}