/*
 * collision_detect
 *  Created on: Sep 21, 2020
 *      Author: olegzilm
 */
#ifndef GLOBAL_UTILS_H_
#define GLOBAL_UTILS_H_

#define _USE_MATH_DEFINES

#include "eigen3/Eigen/Dense"
#include "constants.h"
#include "math/line2d.h"
#include "math/math_util.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include "visualization/visualization.h"
#include "sensor_msgs/LaserScan.h"
#include <string>

// Liyan's xtensor stuff
#include "xtensor/xtensor.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xbuilder.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xindex_view.hpp"
//#include "xsimd/xsimd.hpp"


namespace debug {
    template <typename T>
    void print_line(const geometry::Line<T>& line,std::string prefix = "") {
        std::cout << prefix << " (" << line.p0.x() << ","
             << line.p0.y() << ")-(" << line.p1.x()
             << "," << line.p1.y() << ")" << std::endl;
    }
    void inline print_loc(const Eigen::Vector2f& loc,std::string prefix = "") {
            std::cout << prefix << " (" << loc.x() << "," << loc.y() << ")" << std::endl;
        }
}

namespace visualization {

    inline void DrawCar(float w, float l, uint32_t color,
                        amrl_msgs::VisualizationMsg& msg) {
        Eigen::Vector2f corner1{(l + CarDims::wheelbase) / 2.f, w / 2.f};
        Eigen::Vector2f corner2{(l + CarDims::wheelbase) / 2.f, -w / 2.f};
        Eigen::Vector2f corner3{-(l - CarDims::wheelbase) / 2.f, -w / 2.f};
        Eigen::Vector2f corner4{-(l - CarDims::wheelbase) / 2.f, w / 2.f};
        visualization::DrawLine(corner1, corner2, color, msg);
        visualization::DrawLine(corner2, corner3, color, msg);
        visualization::DrawLine(corner3, corner4, color, msg);
        visualization::DrawLine(corner4, corner1, color, msg);

    }

    inline void DrawPointCloud(const std::vector<Eigen::Vector2f>& p_cloud, uint32_t color,
                                amrl_msgs::VisualizationMsg& msg ) {
        for (size_t i = 0; i < p_cloud.size(); ++i) {
            visualization::DrawPoint(p_cloud[i], color, msg);
        }
    }

}

namespace navigation {
    struct PoseSE2 {
            explicit PoseSE2(): loc{0,0}, angle{0} {};
            explicit PoseSE2(float x,float y,float angle_init):
                    loc{x,y}, angle{angle_init} {};
            explicit PoseSE2(Eigen::Vector2f loc, float angle_init):
                    loc{loc}, angle{angle_init} {};

            Eigen::Vector2f loc;
            float angle;

            const PoseSE2 operator+(const PoseSE2& rhs) {
                PoseSE2 result;
                result.loc = this->loc + rhs.loc;
                result.angle = this->angle + rhs.angle;
                return result;
            }

            PoseSE2& operator+=(const PoseSE2& rhs) {
                this->loc += rhs.loc;
                this->angle += rhs.angle;
                return *this;
            }
    };

    /*const PoseSE2 operator*(double ratio, const PoseSE2& rhs ) {
                return PoseSE2(ratio*(rhs.loc),ratio*(rhs.angle));
    }*/

    // Oleg TODO: consolidate with latencytracker class.
    struct OdomMeasurement {
        float vel;
        float c;
        Eigen::Vector2f loc;
        float angle;
        double timestamp;
    };

    struct ControlCommand {
        explicit ControlCommand(float vel_init,float c_init,double timestamp_init):
                       vel(vel_init),c(c_init),timestamp(timestamp_init) {};
        float vel;
        float c;
        double timestamp;
    };


}

using navigation::PoseSE2;

namespace collision {

    inline bool is_point_in_circle(Eigen::Vector2f circle_center, float radius, Eigen::Vector2f point) {
        if ((circle_center-point).norm() <= radius) {
            return true;
        }
        return false;
    }

    inline bool is_point_in_path(float r, float clearance, const Eigen::Vector2f& point) {
        Eigen::Vector2f turning_center(0,r);
        float collision_ring_internal_r = r-CarDims::w/2-clearance;
        float collision_ring_external_r = r+CarDims::w/2+clearance;
        if (!is_point_in_circle(turning_center,collision_ring_internal_r,point) &&
             is_point_in_circle(turning_center,collision_ring_external_r,point)) {
            return true;
        }
        return false;
    }

    inline float calc_distance_on_curve_from_angle_to_point(float angle, float c) {
        float arc = angle*2;
        if (fabs(c) < GenConsts::kEpsilon) {
            return std::numeric_limits<float>::infinity();
        }
        float r = 1/c;
        float dist = r*arc;
        return dist;

    }

    inline float calc_distance_on_curve_to_point(float c, const Eigen::Vector2f& point) {
        if (fabs(c) < GenConsts::kEpsilon) {
           return point.norm();
        }
        //float angle = atan2(point.y()/point.x());
        float angle = atan(point.y()/point.x());

        return calc_distance_on_curve_from_angle_to_point(angle, c);

    }

    inline float check_collision_curvature_zero(std::vector<Eigen::Vector2f> point_cloud) {
        float upper_y_bound = CarDims::w/2 + CarDims::default_safety_margin;
        float lower_y_bound = -CarDims::w/2 - CarDims::default_safety_margin;
        float fpl = GenConsts::FPL_max_bound;
        for (size_t i = 0; i < point_cloud.size(); ++i) {
            Eigen::Vector2f curr_p = point_cloud[i];
            if (curr_p.y() < upper_y_bound && curr_p.y() > lower_y_bound) {
                fpl = curr_p.x() - CarDims::wheelbase - CarDims::default_safety_margin;
            }
        }
        return fpl;
    }
}

namespace tf {
    inline PoseSE2 transform_pose_to_glob_frame(const PoseSE2& frame_delta,
                                         const PoseSE2& pose_loc_frame) {
        // frame delta is how ahead is the pose of loc frame base from the glob frame
        float new_frame_angle = frame_delta.angle + pose_loc_frame.angle;
        Eigen::Vector2f rotated_displacement =
                Eigen::Rotation2Df(frame_delta.angle) * pose_loc_frame.loc;
        Eigen::Vector2f new_frame_loc = rotated_displacement + frame_delta.loc;
        PoseSE2 new_frame_pose(new_frame_loc,new_frame_angle);
        return new_frame_pose;
    }


    inline PoseSE2 transform_pose_to_loc_frame(const PoseSE2& frame_delta,
                                        const PoseSE2& pose_glob_frame) {
        // frame delta is how ahead is the loc frame (of pose_curr_frame) from the glob frame
        float new_frame_angle = 0;
        Eigen::Vector2f rotated_displacement =
            Eigen::Rotation2Df(-frame_delta.angle) * pose_glob_frame.loc;
        Eigen::Vector2f new_frame_loc =
                rotated_displacement + Eigen::Vector2f(-frame_delta.loc.norm(),0);
        PoseSE2 new_frame_pose(new_frame_loc,new_frame_angle);
        return new_frame_pose;
    }

    inline Eigen::Vector2f transform_point_to_glob_frame(const PoseSE2& frame_delta,
                                           const Eigen::Vector2f& point_loc_frame) {
        // frame_delta is the cooridnates of the loc frame base in the global frame
        Eigen::Vector2f rotated_displacement = Eigen::Rotation2Df(frame_delta.angle) * point_loc_frame;
        Eigen::Vector2f new_frame_loc = rotated_displacement + frame_delta.loc;
        return new_frame_loc;
    }

    inline Eigen::Vector2f transform_point_to_loc_frame(const PoseSE2& frame_delta,
                                          const Eigen::Vector2f& point_glob_frame) {
           // frame_delta is the cooridnates of the loc frame base in the global frame
        Eigen::Vector2f rotated_displacement =
            Eigen::Rotation2Df(-frame_delta.angle) * point_glob_frame;
        Eigen::Vector2f new_frame_loc =
            rotated_displacement + Eigen::Vector2f(-frame_delta.loc.norm(),0);
        return new_frame_loc;
    }

    // OLEG TODO: Ask Lyian to enable this one?
    inline void proj_lidar_2_pts(const sensor_msgs::LaserScan& msg) {
        uint32_t n_pts = msg.ranges.size();
        float end_angle = msg.angle_min + msg.angle_increment * (n_pts - 1);
        auto range_arr = xt::adapt(msg.ranges);
        auto valid_mask = range_arr >= msg.range_min && range_arr <= msg.range_max;
        auto lidar_angles = xt::linspace<float>(msg.angle_min, end_angle, n_pts);
        auto x = xt::cos(lidar_angles) * range_arr;
        auto y = xt::sin(lidar_angles) * range_arr;
        auto pts = xt::stack(xt::xtuple(x, y), 1) + CarDims::laser_loc;
        std::cout << pts << std::endl;
    }

    inline void proj_lidar_2_pts(const sensor_msgs::LaserScan& msg,
                                 std::vector<Eigen::Vector2f>& point_cloud,
                                 const Eigen::Vector2f& kLaserLoc,
                                 bool filter_max_range=false) {
      point_cloud.clear();
      float curr_laser_angle = msg.angle_min;
      for (size_t i = 0; i < msg.ranges.size(); ++i) {
        float curr_range = msg.ranges[i];
        if (curr_range >= msg.range_min && curr_range <= msg.range_max) {
          if (!filter_max_range ||
              curr_range <= (msg.range_max-PhysicsConsts::radar_noise_std)) {
            float x = cos(curr_laser_angle)*curr_range;
            float y = sin(curr_laser_angle)*curr_range;
            Eigen::Vector2f baselink_loc(Eigen::Vector2f(x,y) + kLaserLoc);
            point_cloud.push_back(baselink_loc);
          }
        }
        curr_laser_angle += msg.angle_increment;
      }
    }
}

namespace geometry {
    template <typename T>
    bool is_point_in_circle(const Eigen::Matrix<T,2,1>& center,
                            float radius,
                            const Eigen::Matrix<T,2,1>& point) {
        return (center-point).norm() < radius + GenConsts::kEpsilon;
    }

    template <typename T>
    Line<T> get_sub_segment_in_circle(const Line<T>& segment,
                                      const Eigen::Matrix<T,2,1>& center,
                                      float radius) {

       if (is_point_in_circle(center, radius, segment.p0) &&
                is_point_in_circle(center, radius, segment.p1)) {
           return segment;
       }

       // Calculate intersection points between circle and segment's line.
       Eigen::Matrix<T,2,1> intersect_point_1;
       Eigen::Matrix<T,2,1> intersect_point_2;
       unsigned int num_intersections;
       if (fabs(segment.p1.x()-segment.p0.x()) < GenConsts::kEpsilon) {
           /*
            Solution to equation set
                  r^2 = (x-c_x)^2 + (y-c_y)^2
                  x = b
            is solving the SqEq:
                  y^2 + -2c_y*y+(b-c_x)^2+c_y^2-R^2) = 0
           */
           float y_intersect_1, y_intersect_2;
           float b = segment.p1.x();
           num_intersections = math_util::SolveQuadratic(
                          1.0f,
                          -2*center.y(),
                          (math_util::Sq(b-center.x()) +
                           math_util::Sq(center.y()) -
                           math_util::Sq(radius)),
                          &y_intersect_1, &y_intersect_2);

           if (num_intersections == 2) {
              intersect_point_1 << b,y_intersect_1;
              intersect_point_2 << b,y_intersect_2;
          }
      } else {
           float x_intersect_1, x_intersect_2;
           float a = (segment.p1.y()-segment.p0.y())/(segment.p1.x()-segment.p0.x());
           float b = segment.p1.y() - a*segment.p1.x();
           /*
             Solution to equation set
                   r^2 = (x-c_x)^2 + (y-c_y)^2
                   y = ax+b
             is solving the SqEq:
                   (1+a^2)x^2 + (2ab-2c_x-2c_y*a)x+(c_x^2 + c_y^2 + b^2 - r^2 -2c_y*b = 0
            */
           num_intersections = math_util::SolveQuadratic(
               1+math_util::Sq(a),
               2*(a*b - center.x() - a*center.y()),
               math_util::Sq(center.x())+ math_util::Sq(center.y()) +
               math_util::Sq(b)- math_util::Sq(radius)- 2*b*center.y(),
               &x_intersect_1, &x_intersect_2);

           if (num_intersections == 2) {
               intersect_point_1 << x_intersect_1,a*x_intersect_1+b;
               intersect_point_2 << x_intersect_2,a*x_intersect_2+b;
           }
      }
       Line<T> part_in_circle;
       if (num_intersections < 2) {
           part_in_circle.Set(Eigen::Matrix<T,2,1>(0,0),Eigen::Matrix<T,2,1>(0,0));
       } else {
           if (!is_point_in_circle(center, radius, segment.p0) &&
               !is_point_in_circle(center, radius, segment.p1)) {
               if (IsBetween(segment.p0,segment.p1,
                             intersect_point_1,
                             GenConsts::kEpsilon)
                   && IsBetween(segment.p0,segment.p1,
                                intersect_point_2,
                                GenConsts::kEpsilon)) {
                   part_in_circle.Set(intersect_point_1, intersect_point_2);
               } else {
                   part_in_circle.Set(Eigen::Matrix<T,2,1>(0,0),Eigen::Matrix<T,2,1>(0,0));
               }
           } else {
               Eigen::Matrix<T,2,1> inside_point;
               if (is_point_in_circle(center, radius, segment.p0)) {
                   inside_point = segment.p0;
               } else {
                   inside_point = segment.p1;
               }

               if (IsBetween(segment.p0,segment.p1,
                             intersect_point_1,
                             GenConsts::kEpsilon)) {
                   part_in_circle.Set(inside_point,intersect_point_1);
               } else {
                   part_in_circle.Set(inside_point,intersect_point_2);
               }
           }
       }

       return part_in_circle;
    }
}

#endif // GLOBAL_UTILS_H_
