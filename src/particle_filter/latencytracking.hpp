//
// Created by liyanc on 9/6/20.
//

#ifndef REPO_LATENCYTRACKING_HPP
#define REPO_LATENCYTRACKING_HPP

#include <cmath>
#include <type_traits>
#include <queue>
#include <algorithm>
#include <xtensor/xtensor.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include "shared/constants.h"
#include "plotpublisher.h"
#include "clock.h"

struct VelocityMeasurement {
    float val;
    double timestamp;
};

struct VelocityControlCommand {
    float val;
    double timestamp;
};

template<typename TMeasure, typename TControl, bool IsKeepMeasure>
class LatencyTracking {
public:
    explicit LatencyTracking(PlotPublisher & pub, float tolerance = 0) :
    tol{tolerance},
    is_first{true},
    plot_publisher_{pub}
    {}

    void add_measurements(const TMeasure & msr) {
        measure_list.emplace_back(msr);
        find_matching_measure_control();
    }

    void add_controls(const TControl & ctl) {
        if (control_list.size() < NumConcurrentControlInQueue and
        (control_list.empty() or !is_equal(control_list.back(), ctl))) {
            control_list.emplace_back(ctl);
            plot_publisher_.publish_named_point("Ctrl Pts", Clock::now(), control_list.size());
        }
    }

    void add_controls(const TControl & ctl, PlotPublisher & publisher) {
        if (control_list.size() < NumConcurrentControlInQueue and
        (control_list.empty() or !is_equal(control_list.back(), ctl))) {
            control_list.emplace_back(ctl);
        }
    }

    const std::deque<float>& get_alllatencies() { return latency_list;}

    float estimate_latency(){
        if (is_first or latency_list.empty()) return PhysicsConsts::default_latency;
        else return std::accumulate(latency_list.begin(), latency_list.end(), 0.0) / latency_list.size();
    }


private:
    float tol;
    bool is_first;
    std::deque<TMeasure> measure_list;
    std::deque<TControl> control_list;
    std::deque<float> latency_list;
    const unsigned long NumConcurrentControlInQueue = 8;
    PlotPublisher & plot_publisher_;

    // Compile-time dispatch with SFINAE
    template<typename TLHS, typename TRHS, typename std::enable_if<std::is_scalar<decltype(TLHS::val)>::value and
            std::is_scalar<decltype(TRHS::val)>::value>::type* = nullptr>
            bool is_equal(const TLHS & lhs, const TRHS & rhs) {
                return fabs(lhs.val - rhs.val) <= tol;
            }

    template<typename TLHS, typename TRHS, typename std::enable_if<std::is_class<decltype(TLHS::val)>::value or
            std::is_class<decltype(TRHS::val)>::value>::type* = nullptr>
            bool is_equal(const TLHS & lhs, const TRHS & rhs) {
                return xt::linalg::norm(lhs.val - rhs.val) <= tol;
            }


    bool find_matching_measure_control(){
        bool is_matched = is_equal(control_list.front(), measure_list.back());
        if (is_matched){
            latency_list.emplace_back(measure_list.back().timestamp - control_list.front().timestamp);
            if (control_list.size() > 1) control_list.pop_front();
            if (!IsKeepMeasure) measure_list.clear();
            update_latency_list();
        }
        return is_matched;
    }

    void update_latency_list() {
        if (is_first and latency_list.size() >= Assignment0::discard_latency_num) {
            is_first = false;
            latency_list.clear();
        }
        if (!is_first and latency_list.size() > Assignment0::runavg_latency_num)
            latency_list.pop_front();
        plot_publisher_.publish_named_point("Latency list", Clock::now(), latency_list.size());
    }
};


#endif //REPO_LATENCYTRACKING_HPP
