//
// Created by liyanc on 9/14/20.
//

#ifndef REPO_PLOTPUBLISHER_H
#define REPO_PLOTPUBLISHER_H

#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace {
    ros::Publisher curve_pub_;
}

class PlotPublisher {
public:
    explicit PlotPublisher(ros::NodeHandle *n);
    void publish_named_point(const std::string& name, float x, float y);

};


#endif //REPO_PLOTPUBLISHER_H
