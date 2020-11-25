//
// Created by liyanc on 10/5/20.
//

#include <iterator>
#include "observation_model.h"
#include "xtensor/xtensor.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor/xio.hpp"
#include <iostream>

ObservationModel::ObservationModel(float gamma, float sigma, float d_long, float d_short) : gamma_(gamma), sigma_(sigma), d_long_(d_long), d_short_(d_short) {}

float ObservationModel::calculate_accumulated_loglikelihood(simd_vec_type &intersections, simd_vec_type &observations, float range_min, float range_max) {
    simd_vec_type log_likelihood;
    log_likelihood.resize(intersections.size());
    std::transform(intersections.begin(), intersections.end(), observations.begin(),
                     log_likelihood.begin(),
                     [&](const auto &intersect, const auto & observ){
        float div = 0;
        if((observ > range_max) or (observ < range_min)){
            div = 0;
        }
        else if(observ < (intersect - d_short_)){
            div = d_short_ / sigma_;
        }
        else if(observ > (intersect + d_long_)){
            div = d_long_ / sigma_;
        }
        else{
            div = (intersect - observ) / sigma_;
        }
        
        return div * div;
    });
    std::cout << "Weight vector:" << std::endl;
    std::cout << std::endl << xt::adapt(log_likelihood) << std::endl;
    float acc = xt::sum(xt::adapt(log_likelihood))[0];
    return -acc * gamma_;
}


float ObservationModel::calculate_normalized_weight(){
    return 0;
}

float ObservationModel::calculate_total_weights(){
    return 0;
}

