//
// Created by liyanc on 10/5/20.
//

#ifndef REPO_OBSERVATION_MODEL_H
#define REPO_OBSERVATION_MODEL_H

#include "constants.h"
#include "xsimd/xsimd.hpp"
#include "common_def.h"

class ObservationModel {
public:
    ObservationModel(float gamma, float sigma);

    float calculate_accumulated_loglikelihood(simd_vec_type & intersections, simd_vec_type & observations);

private:
    float gamma, sigma;

};


#endif //REPO_OBSERVATION_MODEL_H
