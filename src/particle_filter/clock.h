//
// Created by liyanc on 9/15/20.
//

#ifndef REPO_CLOCK_H
#define REPO_CLOCK_H

#include "ros/ros.h"

class Clock {
public:
    static float now();
private:
    static double init_t;
    static bool is_inited;
};


#endif //REPO_CLOCK_H
