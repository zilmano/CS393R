//
// Created by liyanc on 11/1/20.
//

#ifndef REPO_CVSHOW_H
#define REPO_CVSHOW_H

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <algorithm>
#include <vector>
#include <string>

template<typename EigenMatrixT>
inline void normalized_imshow(const EigenMatrixT & in, const std::string & title = "Rasterization") {
    cv::Mat cvimg;
    auto maxele = in.maxCoeff();
    Eigen::MatrixXf norm_img = in / maxele;
    std::cout << "Max heatpoint " << maxele << std::endl;
    cv::eigen2cv(norm_img, cvimg);
    cv::imwrite((title + ".png").c_str(), cvimg * 256.0);
    cv::namedWindow(title.c_str(), cv::WINDOW_AUTOSIZE );
    cv::imshow(title.c_str(), cvimg);
}

inline void normalized_voxelshow(const std::vector<Eigen::ArrayXXf> & in, const std::string & title) {
    uint cells = ceil(sqrt(in.size()));
    uint side = in.front().rows();
    Eigen::ArrayXXf acc(cells * side, cells * side);
    uint theta_t = 0;
    for (uint ri = 0; ri < cells; ++ri)
        for (uint ci = 0; ci < cells; ++ci){
            Eigen::ArrayXXf slice(side, side);
            for (uint xi = 0; xi < side; ++xi)
                for (uint yi = 0; yi < side; ++yi)
                    slice(xi, yi) = in[xi](yi, theta_t);
            acc.block(ri * side, ci * side, side, side) = slice;
            theta_t++;
        }
    normalized_imshow(Eigen::exp(acc), title);
}

template<typename EigenMatrixT>
inline void normalized_imwrite(const EigenMatrixT & in, const std::string & title = "Rasterization") {
    cv::Mat cvimg;
    auto maxele = in.maxCoeff();
    Eigen::MatrixXf norm_img = in / maxele;
    cv::eigen2cv(norm_img, cvimg);
    cv::imwrite((title + ".png").c_str(), cvimg * 256.0);
}

#endif //REPO_CVSHOW_H
