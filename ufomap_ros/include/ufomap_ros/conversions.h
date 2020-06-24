#ifndef UFOMAP_ROS_CONVERSIONS_H
#define UFOMAP_ROS_CONVERSIONS_H

#include <ufomap/math/pose6.h>
#include <ufomap/math/quaternion.h>
#include <ufomap/math/vector3.h>
#include <ufomap/point_cloud.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

namespace ufomap
{
// Point clouds
void toUfomap(sensor_msgs::PointCloud2::ConstPtr cloud_in, PointCloud& cloud_out);

void fromUfomap(const PointCloud& cloud_in, sensor_msgs::PointCloud2::Ptr cloud_out);

void toUfomap(sensor_msgs::PointCloud2::ConstPtr cloud_in, PointCloudRGB& cloud_out);

void fromUfomap(const PointCloudRGB& cloud_in, sensor_msgs::PointCloud2::Ptr cloud_out);

void toUfomap(sensor_msgs::PointCloud2::ConstPtr cloud_in, PointCloudI& cloud_out);

void fromUfomap(const PointCloudI& cloud_in, sensor_msgs::PointCloud2::Ptr cloud_out);

// Vector3
void toUfomap(const geometry_msgs::Vector3& vector3_in,
							ufomap_math::Vector3& vector3_out);

ufomap_math::Vector3 toUfomap(const geometry_msgs::Vector3& vector3);

void fromUfomap(const ufomap_math::Vector3& vector3_in,
								geometry_msgs::Vector3& vector3_out);

geometry_msgs::Vector3 fromUfomap(const ufomap_math::Vector3& vector3);

// Quaternion
void toUfomap(const geometry_msgs::Quaternion& quaternion_in,
							ufomap_math::Quaternion& quaternion_out);

ufomap_math::Quaternion toUfomap(const geometry_msgs::Quaternion& quaternion);

void fromUfomap(const ufomap_math::Quaternion& quaternion_in,
								geometry_msgs::Quaternion& quaternion_out);

geometry_msgs::Quaternion fromUfomap(const ufomap_math::Quaternion& quaternion);

// Transforms
void toUfomap(const geometry_msgs::Transform& transform_in,
							ufomap_math::Pose6& transform_out);

ufomap_math::Pose6 toUfomap(const geometry_msgs::Transform& transform);

void fromUfomap(const ufomap_math::Pose6& transform_in,
								geometry_msgs::Transform& transform_out);
								
geometry_msgs::Transform fromUfomap(const ufomap_math::Pose6& transform);

}  // namespace ufomap

#endif  // UFOMAP_ROS_CONVERSIONS_H