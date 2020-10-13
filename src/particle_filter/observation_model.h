//
// Created by liyanc on 10/5/20.
//

#ifndef REPO_OBSERVATION_MODEL_H
#define REPO_OBSERVATION_MODEL_H

#include "shared/constants.h"
#include "xsimd/xsimd.hpp"
#include "common_def.h"

class ObservationModel {
public:
    ObservationModel(float gamma, float sigma, float d_long, float d_short);

    float calculate_accumulated_loglikelihood(simd_vec_type & intersections, simd_vec_type & observations, float range_min, float range_max);

    void setGamma(float gamma) {
      gamma_ = gamma;
    };
    void setSigma(float sigma) {
      sigma_= sigma;
    }
    void setDLong(float d_long){
      d_long_= d_long;
    }
    void setDShort(float d_short){
      d_short_= d_short;
    }

    float calculate_normalized_weight();

    float calculate_total_weights();

private:
    float gamma_;
    float sigma_;
    float d_long_;
    float d_short_;

};


#endif //REPO_OBSERVATION_MODEL_H
