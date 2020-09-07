//
// Created by liyanc on 9/7/20.
//

#ifndef REPO_CONSTANTS_H
#define REPO_CONSTANTS_H

namespace PhysicsConsts{
    const static float max_acc = 3.0f;
    const static float max_vel = 1.0f;
    const static float act_latency_portion = 0.25f;
    const static float default_latency = 50.0f / 1000.0f;
}

namespace Assignment0{
    const static float target_dis = 2.0f;
    const static float timeframe = 1.0f/20.0f;
    const static unsigned long discard_latency_num = 4;
    const static unsigned long runavg_latency_num = 5;
}

#endif //REPO_CONSTANTS_H
