#ifndef UFOMAP_ROS_CONVERSIONS_H
#define UFOMAP_ROS_CONVERSIONS_H

#include <ufomap/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

namespace ufomap
{
void toUfomap(sensor_msgs::PointCloud2::ConstPtr cloud_in, PointCloud& cloud_out);

void fromUfomap(const PointCloud& cloud_in, sensor_msgs::PointCloud2::Ptr cloud_out);


void toUfomap(sensor_msgs::PointCloud2::ConstPtr cloud_in, PointCloudRGB& cloud_out);

void fromUfomap(const PointCloudRGB& cloud_in, sensor_msgs::PointCloud2::Ptr cloud_out);

void toUfomap(sensor_msgs::PointCloud2::ConstPtr cloud_in, PointCloudI& cloud_out);

void fromUfomap(const PointCloudI& cloud_in, sensor_msgs::PointCloud2::Ptr cloud_out);
}  // namespace ufomap

#endif // UFOMAP_ROS_CONVERSIONS_H