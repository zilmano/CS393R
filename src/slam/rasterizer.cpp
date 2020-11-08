//
// Created by liyanc on 10/31/20.
//

#include "2d_normal.h"
#include "rasterizer.h"
#include <eigen3/Eigen/Eigenvalues>

#include "cvshow.h"


Rasterizer::Rasterizer(int res_x, int res_y) : res_x(res_x), res_y(res_y) {
    image.resize(res_x, res_y);
    qry_img.resize(res_x, res_y);
}

void Rasterizer::update_grid_dim(const std::vector<Vector2f> &pts) {
    min_x = INFINITY;
    min_y = INFINITY;
    max_x = -INFINITY;
    max_y = -INFINITY;
    for (auto & p : pts){
        min_x = (min_x < p[0])?min_x:p[0];
        min_y = (min_y < p[1])?min_y:p[1];
        max_x = (max_x > p[0])?max_x:p[0];
        max_y = (max_y > p[1])?max_y:p[1];
    }
    float gap_x = (max_x - min_x) * 0.1;
    float gap_y = (max_y - min_y) * 0.1;
    min_x -= gap_x;
    max_x += gap_x;
    step_x = (max_x - min_x) / res_x;
    min_y -= gap_y;
    max_y += gap_y;
    step_y = (max_y - min_y) / res_y;
}

void Rasterizer::draw_point(const std::vector<Vector2f> &pts, Matrix2f & sigma) {
    Eigen::EigenSolver<Matrix2f> dec{sigma};
    float radius = sqrt(10 * dec.eigenvalues().cwiseAbs().eval().maxCoeff());
    std::vector<Vector2f> coords;
    coords.reserve(res_x * res_y);
    for (auto & p : pts) {
        auto low = coor2idx(p - Vector2f{radius, radius}).cwiseMax(Vector2f{0, 0}).eval();
        auto high = coor2idx(p + Vector2f{radius, radius}).cwiseMin(Vector2f{res_x, res_y}).eval();

        coords.clear();
        for (int x = low[0]; x <= high[0]; ++x)
            for (int y = low[1]; y <= high[1]; ++y)
                coords.emplace_back(idx2coor(x, y));
        auto likelihood = unnormalized_mvn(coords, p, sigma);
        auto iter = likelihood.begin();
        for (int x = low[0]; x <= high[0]; ++x)
            for (int y = low[1]; y <= high[1]; ++y){
                float ll = *(iter++);
                image(x, y) = (image(x, y) > ll)?image(x, y):ll;
            }
    }
    image = ((image / image.sum()).log() + epsilon).eval();
}

inline Vector2f Rasterizer::idx2coor(int ix, int iy) {
    return Vector2f{ix * step_x + min_x, iy * step_y + min_y};
}

inline Vector2f Rasterizer::coor2idx(const Vector2f &coor) {
    Vector2f diff = coor - Vector2f{min_x, min_y};
    return Vector2f{diff[0] / step_x, diff[1] / step_y};
}

decltype(Rasterizer::image) & Rasterizer::rasterize(std::vector<Vector2f> &pts,
                                                    Matrix2f & sigma,
                                                    bool imshow) {
    image.setZero();
    update_grid_dim(pts);
    draw_point(pts, sigma);
    if (imshow)
        normalized_imshow(Eigen::MatrixXf(image.exp().matrix()));
    return image;
}

std::vector<float> Rasterizer::query(const std::vector<Vector2f> &pts) {
    std::vector<float> res;
    res.reserve(pts.size());
    for (auto & pt : pts) {
        if (pt[0] < min_x or pt[0] >= max_x or pt[1] < min_y or pt[1] >= max_y)
            res.emplace_back(log(epsilon));
        else {
            Eigen::Vector2f idx = Eigen::round(coor2idx(pt).array()).eval();
            res.emplace_back(image(static_cast<int>(idx[0]), static_cast<int>(idx[1])));
            qry_img(static_cast<int>(idx[0]), static_cast<int>(idx[1])) += 1;
        }
    }
    return res;
}

decltype(Rasterizer::qry_img) & Rasterizer::get_qry_history(bool reset) {
    if (reset) qry_img.setZero();
    return qry_img;
}
