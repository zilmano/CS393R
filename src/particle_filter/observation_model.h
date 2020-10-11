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
    ObservationModel(float gamma, float sigma);

    float calculate_accumulated_loglikelihood(simd_vec_type & intersections, simd_vec_type & observations);

    void setGamma(float gamma) {
      gamma_ = gamma;
    };
    void setSigma(float sigma) {
      sigma_= sigma;
    }

    float calculate_normalized_weight();

    float calculate_total_weights();

private:
    float gamma_;
    float sigma_;

};


#endif //REPO_OBSERVATION_MODEL_H
