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
		ufomap::PointCloud cloud;

		// auto a1 and a2 are same as:
		// geometry_msgs::TransformStamped tmp_transform =
		// tf_buffer_.lookupTransform(frame_id_, msg->header.frame_id, msg->header.stamp,
		// transform_timeout_);
		// ufomap::toUfomap(msg, cloud);

		auto a1 = std::async(std::launch::async, [this, &msg] {
			return tf_buffer_.lookupTransform(frame_id_, msg->header.frame_id,
																				msg->header.stamp, transform_timeout_);
		});
		auto a2 =
				std::async(std::launch::async, [&msg, &cloud] { ufomap::toUfomap(msg, cloud); });

		ufomap_math::Pose6 transform = ufomap::toUfomap(a1.get().transform);
		a2.wait();
		cloud.transform(transform);
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
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = frame_id_;

	if (0 < map_pub_.getNumSubscribers() || map_pub_.isLatched())
	{
		ufomap_msgs::Ufomap msg;
		ufomap_msgs::mapToMsg(map_, msg, true);
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
}

}  // namespace ufomap_mapping