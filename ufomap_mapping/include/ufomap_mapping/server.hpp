#ifndef UFOMAP_MAPPING_SERVER_H
#define UFOMAP_MAPPING_SERVER_H

#include "ufomap_msgs/msg/ufomap.hpp"

#include "rclcpp/rclcpp.hpp"

#include "ufomap/octree.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

namespace ufomap_mapping
{
class UFOMapServer : public rclcpp::Node
{
public:
	UFOMapServer();

private:
	void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

	void timerCallback();

private:
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

	rclcpp::Publisher<ufomap_msgs::msg::Ufomap>::SharedPtr map_pub_;
	rclcpp::Publisher<ufomap_msgs::msg::Ufomap>::SharedPtr map_binary_pub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

	rclcpp::TimerBase::SharedPtr pub_timer_;

	// TF2
	std::shared_ptr<tf2::BufferCore> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	std::shared_ptr<ufomap::Octree> map_;

	// Configureable variables

	std::string frame_id_;

	float max_range_;

	bool insert_discrete_;
	unsigned int insert_depth_;
	unsigned int insert_n_;
	bool clear_robot_enabled_;

	float robot_height_;
	float robot_radius_;

	float pub_rate_;

  rclcpp::Duration transform_timeout_;

	unsigned int cloud_in_queue_size_;
	unsigned int map_queue_size_;
	unsigned int map_binary_queue_size_;
	unsigned int map_cloud_queue_size_;
};
}  // namespace ufomap_mapping

#endif  // UFOMAP_MAPPING_SERVER_H