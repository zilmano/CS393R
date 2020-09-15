//
// Created by liyanc on 9/14/20.
//

#include "plotpublisher.h"
#include <cstring>

PlotPublisher::PlotPublisher(ros::NodeHandle * n)
{
    curve_pub_ = n->advertise<std_msgs::String>("plotty", 1024);
}

void PlotPublisher::publish_named_point(const std::string & name, float x, float y)
{
    char buf[128];
    std_msgs::String msg;
    sprintf(buf, "%s|%f|%.16e", name.c_str(), x, y);
    msg.data = buf;
    curve_pub_.publish(msg);
}
