#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <ufomap_mapping/server.h>
#include <ufomap_msgs/Ufomap.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include <future>

namespace ufomap_mapping
{
UFOMapServer::UFOMapServer(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
	: nh_(nh)
	, nh_priv_(nh_priv)
	, map_pub_(nh_priv.advertise<ufomap_msgs::Ufomap>(
				"map", nh_priv.param("map_queue_size", 10), nh_priv.param("map_latch", false)))
	, map_binary_pub_(nh_priv.advertise<ufomap_msgs::Ufomap>(
				"map_binary", nh_priv.param("map_binary_queue_size", 10),
				nh_priv.param("map_binary_latch", false)))
	, cloud_pub_(nh_priv.advertise<sensor_msgs::PointCloud2>(
				"map_cloud", nh_priv.param("map_cloud_queue_size", 10),
				nh_priv.param("map_cloud_latch", false)))
	, tf_buffer_(ros::Duration(30))
	, tf_listener_(tf_buffer_)
	, cs_(nh_priv)
	, map_(nh_priv.param("resolution", 0.1), nh_priv.param("depth_levels", 16),
				 !nh_priv.param("multithreaded", false))
{
	// Set up dynamic reconfigure server
	f_ = boost::bind(&UFOMapServer::configCallback, this, _1, _2);
	cs_.setCallback(f_);

	cloud_sub_ = nh.subscribe("cloud_in", nh_priv.param("cloud_in_queue_size", 10),
														&UFOMapServer::cloudCallback, this);

	if (0 < pub_rate_)
	{
		pub_timer_ =
				nh_priv.createTimer(ros::Rate(pub_rate_), &UFOMapServer::timerCallback, this);
	}

	// TODO: Enable services
}

// Private functions
void UFOMapServer::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	try
	{
		ufomap::PointCloudRGB cloud;

		// auto a1 and a2 are same as:
		geometry_msgs::TransformStamped tmp_transform = tf_buffer_.lookupTransform(
				frame_id_, msg->header.frame_id, msg->header.stamp, transform_timeout_);
		sensor_msgs::PointCloud2::Ptr msg_transformed(new sensor_msgs::PointCloud2);
		tf2::doTransform(*msg, *msg_transformed, tmp_transform);

		// ufomap::toUfomap(msg, cloud);
		// ufomap_math::Pose6 transform = ufomap::toUfomap(tmp_transform.transform);

		// auto a1 = std::async(std::launch::async, [this, &msg] {
		// 	return tf_buffer_.lookupTransform(frame_id_, msg->header.frame_id,
		// 																		msg->header.stamp, transform_timeout_);
		// });
		// auto a2 =
		// 		std::async(std::launch::async, [&msg, &cloud] { ufomap::toUfomap(msg, cloud);
		// });
		// auto a2 = std::async(std::launch::async, [&msg_filtered, &cloud] {
		// 	ufomap::toUfomap(msg_filtered, cloud);
		// });

		// ufomap_math::Pose6 transform = ufomap::toUfomap(a1.get().transform);
		// a2.wait();

		// cloud.transform(transform);

		// Filtering
		pcl::PCLPointCloud2::Ptr msg_pcl(new pcl::PCLPointCloud2);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
				new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_filtered(
				new pcl::PointCloud<pcl::PointXYZRGB>);
		sensor_msgs::PointCloud2::Ptr msg_filtered(new sensor_msgs::PointCloud2);

		// Convert to PCL ROS data type
		pcl_conversions::toPCL(*msg_transformed, *msg_pcl);

		// Convert to PCL data type
		pcl::fromPCLPointCloud2(*msg_pcl, *pcl_cloud);

		fprintf(stderr, "First:                             %lu\n", pcl_cloud->size());

		if (enable_voxel_grid_filter_)
		{  // Apply voxel filter
			pcl::VoxelGrid<pcl::PointXYZRGB> sor_voxel;
			sor_voxel.setInputCloud(pcl_cloud);
			sor_voxel.setLeafSize(map_.getResolution(), map_.getResolution(),
														map_.getResolution());
			sor_voxel.filter(*pcl_cloud_filtered);
			*pcl_cloud = *pcl_cloud_filtered;
			fprintf(stderr, "After voxel filter:                %lu\n", pcl_cloud->size());
		}

		if (enable_remove_nan_)
		{
			// Remove NaNs
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*pcl_cloud, *pcl_cloud, indices);
			fprintf(stderr, "After remove NaN:                  %lu\n", pcl_cloud->size());
		}

		if (enable_statistical_outlier_removal_)
		{
			// Apply statistical outlier removal
			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
			sor.setInputCloud(pcl_cloud);
			sor.setMeanK(statistical_outlier_removal_mean_k_);
			sor.setStddevMulThresh(statistical_outlier_removal_stddev_);
			sor.filter(*pcl_cloud_filtered);
			*pcl_cloud = *pcl_cloud_filtered;
			fprintf(stderr, "After statistical outlier removal: %lu\n", pcl_cloud->size());
		}

		if (enable_radius_outlier_removal_)
		{
			// Apply radius outlier removal
			pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> sor;
			sor.setInputCloud(pcl_cloud);
			sor.setRadiusSearch(radius_outlier_removal_radius_);
			sor.setMinNeighborsInRadius(radius_outlier_removal_neighbors_);
			sor.filter(*pcl_cloud_filtered);
			*pcl_cloud = *pcl_cloud_filtered;
			fprintf(stderr, "After radius outlier removal:      %lu\n", pcl_cloud->size());
		}

		// Registration
		if (enable_registration_)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_target_cloud(
					new pcl::PointCloud<pcl::PointXYZRGB>);
			for (auto it = map_.begin_leafs(), it_end = map_.end_leafs(); it != it_end; ++it)
			{
				pcl::PointXYZRGB point;
				point.x = it.getX();
				point.y = it.getY();
				point.z = it.getZ();
				point.r = it->node->color.r;
				point.g = it->node->color.g;
				point.b = it->node->color.b;
				icp_target_cloud->push_back(point);
			}
			icp_target_cloud->width = icp_target_cloud->points.size();
			icp_target_cloud->height = 1;

			if (!icp_target_cloud->empty())
			{
				pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
				icp.setInputSource(pcl_cloud);
				icp.setInputTarget(icp_target_cloud);

				// Set the max correspondence distance to 5cm (e.g., correspondences with higher
				// distances will be ignored)
				icp.setMaxCorrespondenceDistance(icp_correspondence_distance_);
				// Set the maximum number of iterations (criterion 1)
				icp.setMaximumIterations(icp_max_iterations_);
				// Set the transformation epsilon (criterion 2)
				icp.setTransformationEpsilon(icp_transform_epsilon_);
				// Set the euclidean distance difference epsilon (criterion 3)
				icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_);

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr final(
						new pcl::PointCloud<pcl::PointXYZRGB>);
				icp.align(*pcl_cloud);
				// *cloud = *final;
				fprintf(stderr, "After registration:                %lu\n", pcl_cloud->size());
				fprintf(stderr, "Registration has converged: %s, score: %f\n",
								icp.hasConverged() ? "true" : "false", icp.getFitnessScore());
				pcl::transformPointCloud(*pcl_cloud, *pcl_cloud_filtered,
																 icp.getFinalTransformation());
				*pcl_cloud = *pcl_cloud_filtered;
			}
		}

		// Convert to PCL ROS data type
		pcl::toPCLPointCloud2(*pcl_cloud, *msg_pcl);

		// Convert to ROS data type
		pcl_conversions::fromPCL(*msg_pcl, *msg_filtered);

		// Convert to UFOMap data type
		ufomap::toUfomap(msg_filtered, cloud);

		ufomap_math::Pose6 transform = ufomap::toUfomap(tmp_transform.transform);

		if (insert_discrete_)
		{
			map_.insertPointCloudDiscrete(transform.translation(), cloud, max_range_, insert_n_,
																		insert_depth_);
		}
		else
		{
			map_.insertPointCloud(transform.translation(), cloud, max_range_);
		}

		if (clear_robot_enabled_)
		{
			ufomap::Point3 robot_bbx_min(transform.x() - robot_radius_,
																	 transform.y() - robot_radius_,
																	 transform.z() - (robot_height_ / 2.0));
			ufomap::Point3 robot_bbx_max(transform.x() + robot_radius_,
																	 transform.y() + robot_radius_,
																	 transform.z() + (robot_height_ / 2.0));
			map_.clearAreaBBX(robot_bbx_min, robot_bbx_max, 0);
		}
	}
	catch (tf2::TransformException& ex)
	{
		ROS_WARN_THROTTLE(1, "%s", ex.what());
	}
}

void UFOMapServer::timerCallback(const ros::TimerEvent& event)
{
	fprintf(stderr, "Woop woop!\n");
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = frame_id_;

	if (0 < map_pub_.getNumSubscribers() || map_pub_.isLatched())
	{
		ufomap_msgs::Ufomap msg;
		ufomap_msgs::mapToMsg(map_, msg, false);
		msg.header = header;
		map_pub_.publish(msg);
	}

	if (0 < map_binary_pub_.getNumSubscribers() || map_binary_pub_.isLatched())
	{
		ufomap_msgs::Ufomap msg;
		ufomap_msgs::mapToMsg(map_, msg, false, true);
		msg.header = header;
		map_binary_pub_.publish(msg);
	}

	if (0 < cloud_pub_.getNumSubscribers() || cloud_pub_.isLatched())
	{
		ufomap::PointCloud cloud;
		for (auto it = map_.begin_leafs(true, false, false, false, 0),
							it_end = map_.end_leafs();
				 it != it_end; ++it)
		{
			cloud.push_back(it.getCenter());
		}
		sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
		ufomap::fromUfomap(cloud, cloud_msg);
		cloud_msg->header = header;
		cloud_pub_.publish(cloud_msg);
	}
}

void UFOMapServer::configCallback(ufomap_mapping::ServerConfig& config, uint32_t level)
{
	frame_id_ = config.frame_id;
	max_range_ = config.max_range;
	insert_discrete_ = config.insert_discrete;
	insert_depth_ = config.insert_depth;
	insert_n_ = config.insert_n;
	clear_robot_enabled_ = config.clear_robot;
	robot_height_ = config.robot_height;
	robot_radius_ = config.robot_radius;

	if (pub_rate_ != config.pub_rate)
	{
		pub_rate_ = config.pub_rate;
		if (0 < pub_rate_)
		{
			pub_timer_ =
					nh_priv_.createTimer(ros::Rate(pub_rate_), &UFOMapServer::timerCallback, this);
		}
	}

	transform_timeout_.fromSec(config.transform_timeout);

	if (cloud_in_queue_size_ != config.cloud_in_queue_size)
	{
		cloud_sub_ = nh_.subscribe("cloud_in", config.cloud_in_queue_size,
															 &UFOMapServer::cloudCallback, this);
	}

	if (map_pub_.isLatched() != config.map_latch ||
			map_queue_size_ != config.map_queue_size)
	{
		map_pub_ = nh_priv_.advertise<ufomap_msgs::Ufomap>("map", config.map_queue_size,
																											 config.map_latch);
	}

	if (map_binary_pub_.isLatched() != config.map_binary_latch ||
			map_binary_queue_size_ != config.map_binary_queue_size)
	{
		map_binary_pub_ = nh_priv_.advertise<ufomap_msgs::Ufomap>(
				"map_binary", config.map_binary_queue_size, config.map_binary_latch);
	}

	if (cloud_pub_.isLatched() != config.map_cloud_latch ||
			map_cloud_queue_size_ != config.map_cloud_queue_size)
	{
		cloud_pub_ = nh_priv_.advertise<sensor_msgs::PointCloud2>(
				"map_cloud", config.map_cloud_queue_size, config.map_cloud_latch);
	}

	// Filters
	enable_voxel_grid_filter_ = config.enable_voxel_grid_filter;

	enable_remove_nan_ = config.enable_remove_nan;

	enable_statistical_outlier_removal_ = config.enable_statistical_outlier_removal;
	statistical_outlier_removal_mean_k_ = config.statistical_outlier_removal_mean_k;
	statistical_outlier_removal_stddev_ = config.statistical_outlier_removal_stddev;

	enable_radius_outlier_removal_ = config.enable_radius_outlier_removal;
	radius_outlier_removal_radius_ = config.radius_outlier_removal_radius;
	radius_outlier_removal_neighbors_ = config.radius_outlier_removal_neighbors;

	enable_registration_ = config.enable_registration;
	icp_correspondence_distance_ = config.icp_correspondence_distance;
	icp_max_iterations_ = config.icp_max_iterations;
	icp_transform_epsilon_ = config.icp_transform_epsilon;
	icp_euclidean_fitness_epsilon_ = config.icp_euclidean_fitness_epsilon;
}

}  // namespace ufomap_mapping
