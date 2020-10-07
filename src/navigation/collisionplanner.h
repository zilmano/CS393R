//
// Created by liyanc on 9/20/20.
//

#ifndef REPO_COLLISIONPLANNER_H
#define REPO_COLLISIONPLANNER_H

#include "world.h"
#include "xtensor/xtensor.hpp"
#include <vector>
#include "shared/constants.h"
#include "visualization/visualization.h"
#include "amrl_msgs/VisualizationMsg.h"

class CollisionPlanner {
public:
    CollisionPlanner(World &pt_cloud);

    xt::xtensor<float, 2> convert4corner2cspace(float curvature);

    xt::xtensor<float, 1> convert_pts2cspace(const xt::xtensor<float, 1> &pt, float curvature, bool);

    float linear_interpolate_params_wrt_R(const xt::xtensor<float, 1> &pt1, const xt::xtensor<float, 1> &pt2, float Rp);

    std::vector<xt::xtensor<float, 1>> select_potential_collision(float curvature, std::vector<Eigen::Vector2f> & pts);

    float calculate_shortest_collision(float curvature, std::vector<xt::xtensor<float, 1>> & colliding_pts);
    float calculate_shortest_collision_flat(std::vector<Eigen::Vector2f> & pts);
    
    std::vector<float> generate_candidate_paths(float c_step=0.005, float min_radius=2*CarDims::l);

    // OLEG TODO:: to impelement!
    float calc_dist_on_curve_for_angle(float curvature, float angle);
    float calculate_shortest_translational_displacement(float curvature, std::vector<Eigen::Vector2f> & pts);

    float calculate_shortest_collision(float curvature, std::vector<xt::xtensor<float, 1>> & colliding_pts,
                                       amrl_msgs::VisualizationMsg& msg);


private:
    World &point_cloud_;
};


#endif //REPO_COLLISIONPLANNER_H
