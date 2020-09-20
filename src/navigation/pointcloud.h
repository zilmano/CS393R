//
// Created by liyanc on 9/18/20.
//

#ifndef REPO_POINTCLOUD_H
#define REPO_POINTCLOUD_H

#include "sensor_msgs/LaserScan.h"

class PointCloud {

};

void ProjLidar2Pts(const sensor_msgs::LaserScan& msg);

#endif //REPO_POINTCLOUD_H
