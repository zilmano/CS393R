//
// Created by liyanc on 10/5/20.
//

#include <iterator>
#include "observation_model.h"
#include "xtensor/xtensor.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor/xio.hpp"
#include <iostream>

ObservationModel::ObservationModel(float gamma, float sigma) : gamma_(gamma), sigma_(sigma) {}

float ObservationModel::calculate_accumulated_loglikelihood(simd_vec_type &intersections, simd_vec_type &observations) {
    simd_vec_type log_likelihood;
    log_likelihood.resize(intersections.size());
    xsimd::transform(intersections.begin(), intersections.end(), observations.begin(),
                     log_likelihood.begin(),
                     [&](const auto &intersect, const auto & observ){
        auto div = (intersect - observ) / sigma_;
        return div * div;
    });
    std::cout << std::endl << xt::adapt(log_likelihood) << std::endl;
    float acc = xt::sum(xt::adapt(log_likelihood))[0];
    return acc * -gamma_;
}