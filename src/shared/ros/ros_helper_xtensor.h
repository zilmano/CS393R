//
// Created by liyanc on 9/6/20.
//

#ifndef REPO_ROS_HELPER_XTENSOR_H
#define REPO_ROS_HELPER_XTENSOR_H

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "xtensor/xtensor.hpp"

namespace ros_helpers{

    template<typename Derived>
    geometry_msgs::Point XTensor3DToRosPoint(const xt::xarray<Derived>& v) {
        geometry_msgs::Point p{.x = v[0], .y = v[1], .z = v[2]};
        return p;
    }

    template<typename Derived>
    geometry_msgs::Point XTensor2DToRosPoint(const xt::xarray<Derived>& v) {
        geometry_msgs::Point p{.x = v[0], .y = v[1], .z = 0};
        return p;
    }
}

#endif //REPO_ROS_HELPER_XTENSOR_H
