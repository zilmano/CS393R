//
// Created by liyanc on 10/31/20.
//

#ifndef REPO_2D_NORMAL_H
#define REPO_2D_NORMAL_H

#include <vector>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using Eigen::Vector2f;
using Eigen::Matrix2f;

std::vector<float> loglikelihood_mvn(const std::vector<Vector2f> & x, const Vector2f & mu, const Matrix2f & sigma) {
    Matrix2f precision = sigma.inverse();
    Eigen::CompleteOrthogonalDecomposition<Matrix2f> dec{sigma};
    float logabsdet = dec.logAbsDeterminant();
    const float norm_const = log(M_PI * 2) - 0.5 * logabsdet;

    std::vector<float> res;
    res.reserve(x.size());
    for (auto & obs : x) {
        auto diff = obs - mu;
        float mahalanobis = diff.transpose() * precision * diff;
        res.emplace_back(norm_const - mahalanobis);
    }
    return res;
}

std::vector<float> unnormalized_mvn(const std::vector<Vector2f> & x, const Vector2f & mu, const Matrix2f & sigma) {
    Matrix2f precision = sigma.inverse();

    std::vector<float> res;
    res.reserve(x.size());
    for (auto & obs : x) {
        auto diff = obs - mu;
        float mahalanobis = diff.transpose() * precision * diff;
        res.emplace_back(exp(-0.5 * mahalanobis));
    }
    return res;
}


#endif //REPO_2D_NORMAL_H
