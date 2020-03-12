#ifndef UFOMAP_MAPPING_SERVER_H
#define UFOMAP_MAPPING_SERVER_H
#include <ros/ros.h>

#include <ufomap/octree.h>

#include <ufomap_mapping/ServerConfig.h>

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace ufomap_server
{
class UFOMapServer
{
public:
	UFOMapServer();

private:
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

	void timerCallback(const ros::TimerEvent& event);

	void configCallback(ufomap_server::ServerConfig& config, uint32_t level);

private:
	ros::Subscriber cloud_sub_;

	ros::Publisher map_pub_;
	ros::Publisher map_binary_pub_;
	ros::Publisher cloud_pub_;

	ros::Timer pub_timer_;

	ros::ServiceServer get_map_server_;
	ros::ServiceServer clear_area_server_;
	ros::ServiceServer reset_server_;

	// TF2
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	float max_range_;

	std::string frame_id_;

	bool discrete_insert_;
	bool depth_insert_;
	bool n_insert_;
};
}  // namespace ufomap_server

#endif  // UFOMAP_MAPPING_SERVER_H