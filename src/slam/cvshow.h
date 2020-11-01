//
// Created by liyanc on 11/1/20.
//

#ifndef REPO_CVSHOW_H
#define REPO_CVSHOW_H

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

template<typename EigenMatrixT>
void normalized_imshow(const EigenMatrixT & in) {
    cv::Mat cvimg;
    auto maxele = in.maxCoeff();
    Eigen::MatrixXf norm_img = in / maxele;
    std::cout << maxele << std::endl;
    cv::eigen2cv(norm_img, cvimg);
    cv::imwrite("out.png", cvimg * 256.0);
    cv::namedWindow("Rasterization", cv::WINDOW_AUTOSIZE );
    cv::imshow("Rasterization", cvimg);
    cv::waitKey(0);
}

#endif //REPO_CVSHOW_H
