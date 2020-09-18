/*
 * movement_planning.h
 *
 *  Created on: Sep 17, 2020
 *      Author: Oleg Zilman
 */

#ifndef SRC_NAVIGATION_WORLD_H_
#define SRC_NAVIGATION_WORLD_H_


#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include <vector>
#include <memory>

class World {
public:
	/*
	 * class to track all the what we know about the world, the coordinates of obstacles in odom and map frame, map, etc.
	 * for now, includes the down-sampling functionality and constants, that later could be moved to a 'SensorProcessor' class.
	 */

	typedef std::vector<std::vector<Eigen::Vector2f>> ObstacleData;
	explicit World(unsigned long laser_spatial_downsampe_rate,unsigned long laser_time_downsampe_rate):
			      laster_spatial_downsampe_rate_(laser_spatial_downsampe_rate),
				  laster_time_downsample_rate_(laser_time_downsampe_rate),
				  time_sample_counter_(0) {};

	std::vector<Eigen::Vector2f>& get_latest_point_cloud_odom_frame() { return latest_pcloud_odom_frame_; };
	std::vector<Eigen::Vector2f>& get_latest_point_cloud_map_frame() { throw("Not implemented yet.");}

	void update_obsatcle_data_odom_frame(const std::vector<Eigen::Vector2f>& point_cloud) {

		// OLEG TODO:: Introduce pointer to make this whole this faster instead of using copy constructors all the time.
		//             For now, it is OK.
		latest_pcloud_odom_frame_ = point_cloud;
		// OLEG TBD::
		//
		// if (time_sample_counter_ % (laster_time_downsample_rate_-1) == 0) {
	    //   obstacle_data_.push_back(down_sample_point_cloud(point_cloud));
		//   time_sample_counter_ = 0;
		// } else {
		//   ++time_sample_counter_;
		// }
	}

	void update_obbsatcle_data_map_frame(std::vector<Eigen::Vector2f> point_cloud) { throw("Not implemented yet.");}

private:
	std::vector<Eigen::Vector2f> down_sample_point_cloud(std::vector<Eigen::Vector2f>) { throw("Not implemented yet."); };


private:
	std::vector<Eigen::Vector2f> latest_pcloud_odom_frame_;
	ObstacleData obstacle_data_;

	// rates
	unsigned long laster_spatial_downsampe_rate_;
	unsigned long laster_time_downsample_rate_;

	unsigned int time_sample_counter_;
};




#endif /* SRC_NAVIGATION_WORLD_PLANNING_H_ */
