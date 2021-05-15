#ifndef UFOMAP_ROS_CONVERSIONS_H
#define UFOMAP_ROS_CONVERSIONS_H

#include "ufomap/math/pose6.h"
#include "ufomap/math/quaternion.h"
#include "ufomap/math/vector3.h"
#include "ufomap/point_cloud.h"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ufomap
{
// Point clouds
void toUfomap(sensor_msgs::msg::PointCloud2::ConstPtr cloud_in, PointCloud& cloud_out);

void fromUfomap(const PointCloud& cloud_in, sensor_msgs::msg::PointCloud2::Ptr cloud_out);

void toUfomap(sensor_msgs::msg::PointCloud2::ConstPtr cloud_in, PointCloudRGB& cloud_out);

void fromUfomap(const PointCloudRGB& cloud_in, sensor_msgs::msg::PointCloud2::Ptr cloud_out);

void toUfomap(sensor_msgs::msg::PointCloud2::ConstPtr cloud_in, PointCloudI& cloud_out);

void fromUfomap(const PointCloudI& cloud_in, sensor_msgs::msg::PointCloud2::Ptr cloud_out);

// Vector3
void toUfomap(const geometry_msgs::msg::Vector3& vector3_in,
							ufomap_math::Vector3& vector3_out);

ufomap_math::Vector3 toUfomap(const geometry_msgs::msg::Vector3& vector3);

void fromUfomap(const ufomap_math::Vector3& vector3_in,
								geometry_msgs::msg::Vector3& vector3_out);

geometry_msgs::msg::Vector3 fromUfomap(const ufomap_math::Vector3& vector3);

// Quaternion
void toUfomap(const geometry_msgs::msg::Quaternion& quaternion_in,
							ufomap_math::Quaternion& quaternion_out);

ufomap_math::Quaternion toUfomap(const geometry_msgs::msg::Quaternion& quaternion);

void fromUfomap(const ufomap_math::Quaternion& quaternion_in,
								geometry_msgs::msg::Quaternion& quaternion_out);

geometry_msgs::msg::Quaternion fromUfomap(const ufomap_math::Quaternion& quaternion);

// Transforms
void toUfomap(const geometry_msgs::msg::Transform& transform_in,
							ufomap_math::Pose6& transform_out);

ufomap_math::Pose6 toUfomap(const geometry_msgs::msg::Transform& transform);

void fromUfomap(const ufomap_math::Pose6& transform_in,
								geometry_msgs::msg::Transform& transform_out);
								
geometry_msgs::msg::Transform fromUfomap(const ufomap_math::Pose6& transform);

}  // namespace ufomap

#endif  // UFOMAP_ROS_CONVERSIONS_H