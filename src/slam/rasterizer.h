//
// Created by liyanc on 10/31/20.
//

#ifndef REPO_RASTERIZER_H
#define REPO_RASTERIZER_H

#include <eigen3/Eigen/Dense>
#include <vector>

using Eigen::Vector2f;
using Eigen::Matrix2f;

class Rasterizer {
private:
    Eigen::ArrayXXf image;
    float min_x, max_x, step_x, min_y, max_y, step_y;
    int res_x, res_y;

    void update_grid_dim(const std::vector<Vector2f> & pts);
    void draw_point(const std::vector<Vector2f> & pts, Matrix2f & sigma);
    inline Vector2f coor2idx(const Vector2f & coor);
    inline Vector2f idx2coor(int ix, int iy);

public:
    Rasterizer(int res_x, int res_y);
    decltype(image) & rasterize(std::vector<Vector2f> & pts, Matrix2f & sigma);
    std::vector<float> query(std::vector<Vector2f> & pts);
};


#endif //REPO_RASTERIZER_H
