//
// Created by liyanc on 10/31/20.
//

#include "rasterizer.h"
#include "2d_normal.h"
#include <eigen3/Eigen/Eigenvalues>

#include "cvshow.h"


Rasterizer::Rasterizer(int res_x, int res_y) : res_x(res_x), res_y(res_y) {
    image.resize(res_x, res_y);}

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
    float radius = sqrt(5 * dec.eigenvalues().cwiseAbs().eval().maxCoeff());
    std::cout << radius << std::endl;
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
            for (int y = low[1]; y <= high[1]; ++y)
                image(x, y) += *(iter++);
    }
    image = (image / image.sum()).log().eval();
}

inline Vector2f Rasterizer::idx2coor(int ix, int iy) {
    return Vector2f{ix * step_x + min_x, iy * step_y + min_y};
}

inline Vector2f Rasterizer::coor2idx(const Vector2f &coor) {
    Vector2f diff = coor - Vector2f{min_x, min_y};
    return Vector2f{diff[0] / step_x, diff[1] / step_y};
}

decltype(Rasterizer::image) & Rasterizer::rasterize(std::vector<Vector2f> &pts, Matrix2f & sigma) {
    image.setZero();
    update_grid_dim(pts);
    draw_point(pts, sigma);
    normalized_imshow(Eigen::MatrixXf(image.exp().matrix()));
    return image;
}