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

using Eigen::Vector2f;
using amrl_msgs::VisualizationMsg;


namespace visualization {

    inline void DrawCar(float w, float l, uint32_t color, VisualizationMsg& msg) {
        Vector2f corner1{(l + CarDims::wheelbase) / 2.f, w / 2.f};
        Vector2f corner2{(l + CarDims::wheelbase) / 2.f, -w / 2.f};
        Vector2f corner3{-(l - CarDims::wheelbase) / 2.f, -w / 2.f};
        Vector2f corner4{-(l - CarDims::wheelbase) / 2.f, w / 2.f};
        visualization::DrawLine(corner1, corner2, color, msg);
        visualization::DrawLine(corner2, corner3, color, msg);
        visualization::DrawLine(corner3, corner4, color, msg);
        visualization::DrawLine(corner4, corner1, color, msg);

    }

    inline void DrawPointCloud(const std::vector<Vector2f>& p_cloud, uint32_t color,
                               VisualizationMsg& msg ) {
        for (size_t i = 0; i < p_cloud.size(); ++i) {
            visualization::DrawPoint(p_cloud[i], color, msg);
        }
    }

}

namespace navigation {
    struct PoseSE2 {
            inline explicit PoseSE2(): loc{0,0}, angle{0} {};
            inline explicit PoseSE2(float x,float y,float angle_init):
                    loc{x,y}, angle{angle_init} {};
            inline explicit PoseSE2(Vector2f loc, float angle_init):
                    loc{loc}, angle{angle_init} {};

            Vector2f loc;
            float angle;

            inline const PoseSE2 operator+(const PoseSE2& rhs) {
                PoseSE2 result;
                result.loc = this->loc + rhs.loc;
                result.angle = this->angle + rhs.angle;
                return result;
            }

            inline PoseSE2& operator+=(const PoseSE2& rhs) {
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
        inline explicit ControlCommand(float vel_init,float c_init,double timestamp_init):
                       vel(vel_init),c(c_init),timestamp(timestamp_init) {};
        float vel;
        float c;
        double timestamp;
    };


}

using navigation::PoseSE2;

namespace collision {

    inline bool is_point_in_circle(Vector2f circle_center, float radius, Vector2f point) {
        if ((circle_center-point).norm() <= radius) {
            return true;
        }
        return false;
    }

    inline bool is_point_in_path(float r, float clearance, const Vector2f& point) {
        Vector2f turning_center(0,r);
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

    inline float calc_distance_on_curve_to_point(float c, const Vector2f& point) {
        if (fabs(c) < GenConsts::kEpsilon) {
           return point.norm();
        }
        //float angle = atan2(point.y()/point.x());
        float angle = atan(point.y()/point.x());

        return calc_distance_on_curve_from_angle_to_point(angle, c);

    }

    inline float check_collision_curvature_zero(std::vector<Vector2f> point_cloud) {
        float upper_y_bound = CarDims::w/2 + CarDims::default_safety_margin;
        float lower_y_bound = -CarDims::w/2 - CarDims::default_safety_margin;
        float fpl = GenConsts::FPL_max_bound;
        for (size_t i = 0; i < point_cloud.size(); ++i) {
            Vector2f curr_p = point_cloud[i];
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
        Vector2f rotated_displacement =
                Eigen::Rotation2Df(frame_delta.angle) * pose_loc_frame.loc;
        Vector2f new_frame_loc = rotated_displacement + frame_delta.loc;
        PoseSE2 new_frame_pose(new_frame_loc,new_frame_angle);
        return new_frame_pose;
    }


    inline PoseSE2 transform_pose_to_loc_frame(const PoseSE2& frame_delta,
                                        const PoseSE2& pose_glob_frame) {
        // frame delta is how ahead is the loc frame (of pose_curr_frame) from the glob frame
        float new_frame_angle = 0;
        Vector2f rotated_displacement =
            Eigen::Rotation2Df(-frame_delta.angle) * pose_glob_frame.loc;
        Vector2f new_frame_loc =
                rotated_displacement + Vector2f(-frame_delta.loc.norm(),0);
        PoseSE2 new_frame_pose(new_frame_loc,new_frame_angle);
        return new_frame_pose;
    }

    inline Vector2f transform_point_to_glob_frame(const PoseSE2& frame_delta,
                                           const Vector2f& point_loc_frame) {
        // frame_delta is the cooridnates of the loc frame base in the global frame
        Vector2f rotated_displacement = Eigen::Rotation2Df(frame_delta.angle) * point_loc_frame;
        Vector2f new_frame_loc = rotated_displacement + frame_delta.loc;
        return new_frame_loc;
    }

    inline Vector2f transform_point_to_loc_frame(const PoseSE2& frame_delta,
                                          const Vector2f& point_glob_frame) {
           // frame_delta is the cooridnates of the loc frame base in the global frame
        Vector2f rotated_displacement =
            Eigen::Rotation2Df(-frame_delta.angle) * point_glob_frame;
        Vector2f new_frame_loc =
            rotated_displacement + Vector2f(-frame_delta.loc.norm(),0);
        return new_frame_loc;
    }

}

/*
namespace geometry {
    using std::pow;
    template <typename T>
    inline bool is_point_in_circle(const Vector2f& center,float radius,Vector2f point) {
        return (center-point).norm() < radius + GenConsts::kEpsilon;
    }

    inline Line<float> get_sub_segment_in_circle(const Line<float>& segment,
                                                 const Vector2f& center,
                                                 float radius) {

       if (is_point_in_circle(center, radius, segment.p0) &&
                is_point_in_circle(center, radius, segment.p1))
           return segment;

       // Calculate intersection points between circle and segment's line.
       float a = (segment.p1.y()-segment.p0.y()/segment.p1.x()-segment.p0.x());
       float b = segment.p1.y() - a*segment.p1.x();
       // Solution to equation set
       // r^2 = (x-c_x)^2 + (y-c_y)^2
       // y = ax+b
       // is solving the SqEq (1+a)x^2 + (2ab-2c_x-2c_y*a)x+(c_x^2 + c_y^2 + b^2 - r^2 -2c_y*b = 0
       float x_intersect_1, x_intersect_2;
       auto num_intersections = math_util::SolveQuadratic(
           1+a,
           2(a*b - center.x() - a*center.y()),
           pow(center.x(),2)+ pow(center.y(),2) + pow(b,2)- pow(radius,2)- 2*b*center.y(),
           &x_intersect_1, &x_intersect_2);

       if (num_intersections < 2) {
           return Line<float>(Vector2f(0,0),Vector2f(0,0));
       } else {
           Vector2f intersect_point_1(x_intersect_1,a*x_intersect_1+b);
           Vector2f intersect_point_2(x_intersect_2,a*x_intersect_1+b);
           Line<float> part_in_circle;

           if (!is_point_in_circle(center, radius, segment.p0) &&
               !is_point_in_circle(center, radius, segment.p1)) {
              part_in_circle.Set(intersect_point_1, intersect_point_2);
           } else {
               Vector2f inside_point;
               if (is_point_in_circle(center, radius, segment.p0)) {
                   inside_point = segment.p0;
               } else {
                   inside_point = segment.p1;
               }

               if (IsBetween(segment.p0,segment.p1,intersect_point_1,GenConsts::kEpsilon)) {
                   part_in_circle.Set(inside_point,intersect_point_1);
               } else {
                   part_in_circle.Set(inside_point,intersect_point_2);
               }
          }
        return part_in_circle;
      }

}*/

#endif // GLOBAL_UTILS_H_
