//
// Created by liyanc on 10/31/20.
//

#ifndef REPO_2D_NORMAL_H
#define REPO_2D_NORMAL_H

#include <vector>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Matrix2f;
using Eigen::Matrix3f;

inline std::vector<float> loglikelihood_mvn(const std::vector<Vector2f> & x, const Vector2f & mu, const Matrix2f & sigma) {
    Matrix2f precision = sigma.inverse();
    Eigen::CompleteOrthogonalDecomposition<Matrix2f> dec{sigma};
    float logabsdet = dec.logAbsDeterminant();
    const float norm_const = -logf(M_PI * 2) - 0.5 * logabsdet;

    std::vector<float> res;
    res.reserve(x.size());
    for (auto & obs : x) {
        auto diff = obs - mu;
        float mahalanobis = diff.transpose() * precision * diff;
        res.emplace_back(norm_const - mahalanobis);
    }
    return res;
}

inline std::vector<float> loglikelihood_3d_mvn(const std::vector<Vector3f> & x, const Vector3f & mu, const Matrix3f & sigma) {
    Matrix3f precision = sigma.inverse();
    Eigen::CompleteOrthogonalDecomposition<Matrix3f> dec{sigma};
    float logabsdet = dec.logAbsDeterminant();
    const float norm_const = -1.5f * logf(M_PI * 2) - 0.5 * logabsdet;

    std::vector<float> res;
    res.reserve(x.size());
    for (auto & obs : x) {
        auto diff = obs - mu;
        float mahalanobis = diff.transpose() * precision * diff;
        res.emplace_back(norm_const - mahalanobis);
    }
    return res;
}

inline std::vector<float> unnormalized_mvn(const std::vector<Vector2f> & x, const Vector2f & mu, const Matrix2f & sigma) {
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
