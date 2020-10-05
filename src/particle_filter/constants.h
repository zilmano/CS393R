//
// Created by liyanc on 9/7/20.
//

#ifndef REPO_CONSTANTS_H
#define REPO_CONSTANTS_H

#include "xtensor/xtensor.hpp"

namespace GenConsts{
    const float kEpsilon = 1e-5f;
    const float step_period = 0.02f;
    const float FPL_max_bound = 50;
}

namespace PhysicsConsts{
    const static float max_acc = 3.0f;
    const static float max_vel = 1.0f;
    const static float act_latency_portion = 0.25f;
    const static float default_latency = 50.0f / 1000.0f;
}

namespace SamplingConsts {
    const static float downsample_rate_time = 20;
    const static float downsample_rate_space = 5;
}

namespace CarDims{
	// OLEG TODO: mesure the car to see if these are correct.
    const static float w = 0.281f;
	const static float l = 0.535f;
	const static float wheelbase = 0.324f;
	const static float default_safety_margin = 0.15f;
    const static xt::xtensor<float, 2> laser_loc;
}

namespace Assignment0{
    const static float target_dis = 2.0f;
    const static float timeframe = 1.0f/20.0f;
    const static unsigned long discard_latency_num = 1;
    const static unsigned long runavg_latency_num = 6;
}


#endif //REPO_CONSTANTS_H
