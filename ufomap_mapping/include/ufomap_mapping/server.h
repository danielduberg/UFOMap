#ifndef UFOMAP_MAPPING_SERVER_H
#define UFOMAP_MAPPING_SERVER_H
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <ufomap/octree.h>
#include <ufomap/octree_rgb.h>
#include <ufomap_mapping/ServerConfig.h>

namespace ufomap_mapping
{
class UFOMapServer
{
public:
	UFOMapServer(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

	void timerCallback(const ros::TimerEvent& event);

	void configCallback(ufomap_mapping::ServerConfig& config, uint32_t level);

private:
	ros::NodeHandle& nh_;
	ros::NodeHandle& nh_priv_;

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

	// Dynamic reconfigure
	dynamic_reconfigure::Server<ufomap_mapping::ServerConfig> cs_;
	dynamic_reconfigure::Server<ufomap_mapping::ServerConfig>::CallbackType f_;

	ufomap::OctreeRGB map_;

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

	ros::Duration transform_timeout_;

	unsigned int cloud_in_queue_size_;
	unsigned int map_queue_size_;
	unsigned int map_binary_queue_size_;
	unsigned int map_cloud_queue_size_;

	// Different filters
	bool enable_voxel_grid_filter_;

	bool enable_remove_nan_;

	bool enable_statistical_outlier_removal_;
	int statistical_outlier_removal_mean_k_;
	double statistical_outlier_removal_stddev_;

	bool enable_radius_outlier_removal_;
	double radius_outlier_removal_radius_;
	int radius_outlier_removal_neighbors_;

	// Registration
	bool enable_registration_;
	double icp_correspondence_distance_;
	int icp_max_iterations_;
	double icp_transform_epsilon_;
	double icp_euclidean_fitness_epsilon_;
};
}  // namespace ufomap_mapping

#endif  // UFOMAP_MAPPING_SERVER_H