//
// Created by liyanc on 9/15/20.
//

#include "clock.h"


double Clock::init_t = 0.0f;
bool Clock::is_inited = false;

float Clock::now() {
    if (is_inited)
        return static_cast<float>(ros::Time::now().toSec() - init_t);
    else {
        init_t = ros::Time::now().toSec();
        is_inited = true;
        return 0.0f;
    }
}