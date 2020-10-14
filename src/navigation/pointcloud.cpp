//
// Created by liyanc on 9/18/20.
//

#include "pointcloud.h"
#include "shared/constants.h"
#include "xtensor/xtensor.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xbuilder.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xindex_view.hpp"
//#include "xsimd/xsimd.hpp"

#include <iostream>

void ProjLidar2Pts(const sensor_msgs::LaserScan& msg) {
    uint32_t n_pts = msg.ranges.size();
    float end_angle = msg.angle_min + msg.angle_increment * (n_pts - 1);
    auto range_arr = xt::adapt(msg.ranges);
    auto valid_mask = range_arr >= msg.range_min && range_arr <= msg.range_max;
    auto lidar_angles = xt::linspace<float>(msg.angle_min, end_angle, n_pts);
    auto x = xt::cos(lidar_angles) * range_arr;
    auto y = xt::sin(lidar_angles) * range_arr;
    auto pts = xt::stack(xt::xtuple(x, y), 1) + CarDims::laser_loc;
    std::cout << pts << std::endl;
}
