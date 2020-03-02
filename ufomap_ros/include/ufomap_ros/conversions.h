#ifndef UFOMAP_ROS_CONVERSIONS_H
#define UFOMAP_ROS_CONVERSIONS_H

#include <ufomap/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

namespace ufomap
{
/**
 * @brief Converts ROS point cloud msg to UFOMap point cloud
 *
 * @code{.cpp}
	void callback(const sensor_msgs::PointCloud2::ConstPtr& ros_cloud)
	{
		ufomap::PointCloud ufo_cloud;
		ufomap::toUfomap(ros_cloud, ufo_cloud);
	}
 * @endcode
 *
 * @param cloud_in The ROS point cloud msg
 * @param cloud_out The UFOMap point cloud
 */
void toUfomap(sensor_msgs::PointCloud2::ConstPtr cloud_in, PointCloud& cloud_out);

/**
 * @brief Converts UFOMap point cloud to ROS point cloud msg
 *
 * @param cloud_in The UFOMap point cloud
 * @param cloud_out The ROS point cloud msg
 */
void fromUfomap(const PointCloud& cloud_in, sensor_msgs::PointCloud2::Ptr cloud_out);

/**
 * @brief Converts ROS RGB point cloud msg to UFOMap RGB point cloud
 *
 * @code{.cpp}
	void callback(const sensor_msgs::PointCloud2::ConstPtr& ros_cloud)
	{
		ufomap::PointCloudRGB ufo_cloud;
		ufomap::toUfomap(ros_cloud, ufo_cloud);
	}
 * @endcode
 *
 * @param cloud_in The ROS RGB point cloud msg
 * @param cloud_out The UFOMap RGB point cloud
 */
void toUfomap(sensor_msgs::PointCloud2::ConstPtr cloud_in, PointCloudRGB& cloud_out);

/**
 * @brief Converts UFOMap RGB point cloud to ROS RGB point cloud msg
 *
 * @param cloud_in The UFOMap RGB point cloud
 * @param cloud_out The ROS RGB point cloud msg
 */
void fromUfomap(const PointCloudRGB& cloud_in, sensor_msgs::PointCloud2::Ptr cloud_out);

/**
 * @brief Converts ROS intensity point cloud msg to UFOMap intensity point cloud
 *
 * @code{.cpp}
	void callback(const sensor_msgs::PointCloud2::ConstPtr& ros_cloud)
	{
		ufomap::PointCloudI ufo_cloud;
		ufomap::toUfomap(ros_cloud, ufo_cloud);
	}
 * @endcode
 *
 * @param cloud_in The ROS intensity point cloud msg
 * @param cloud_out The UFOMap intensity point cloud
 */
void toUfomap(sensor_msgs::PointCloud2::ConstPtr cloud_in, PointCloudI& cloud_out);

/**
 * @brief Converts UFOMap intensity point cloud to ROS intensity point cloud msg
 *
 * @param cloud_in The UFOMap intensity point cloud
 * @param cloud_out The ROS intensity point cloud msg
 */
void fromUfomap(const PointCloudI& cloud_in, sensor_msgs::PointCloud2::Ptr cloud_out);
}  // namespace ufomap

#endif  // UFOMAP_ROS_CONVERSIONS_H