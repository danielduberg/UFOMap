#include <future>

#include "ufomap_mapping/server.hpp"

#include "ufomap_msgs/msg/ufomap.hpp"
#include "ufomap_msgs/conversions.h"
#include "ufomap_ros/conversions.h"
#include "tf2_ros/buffer_interface.h"


using std::placeholders::_1;

namespace ufomap_mapping
{

UFOMapServer::UFOMapServer()
: Node("ufomap_server"),
  transform_timeout_(0)
{
  // Parameters declaration
  declare_parameter("map_queue_size", 1);
  declare_parameter("map_latch", false);
  declare_parameter("map_binary_queue_size", 1);
  declare_parameter("map_binary_latch", false);
  declare_parameter("map_cloud_queue_size", 1);
  declare_parameter("map_cloud_latch", false);
  declare_parameter("resolution", 0.1);
  declare_parameter("depth_levels", 16);
  declare_parameter("multithreaded", false);
  declare_parameter("frame_id", "map");
  declare_parameter("max_range", 7.0);
  declare_parameter("insert_discrete", true);
  declare_parameter("insert_depth", 0);
  declare_parameter("insert_n", 0);
  declare_parameter("clear_robot", true);
  declare_parameter("robot_height", 0.2);
  declare_parameter("robot_radius", 0.5);
  declare_parameter("pub_rate", 1.0);
  declare_parameter("transform_timeout", 0.1);
  declare_parameter("cloud_in_queue_size", 10);

  // Parameters reading
  frame_id_ = get_parameter("frame_id").as_string();
  max_range_ = get_parameter("max_range").as_double();
  insert_discrete_ = get_parameter("insert_discrete").as_bool();
  insert_depth_ = get_parameter("insert_depth").as_int();
  insert_n_ = get_parameter("insert_n").as_int();
  clear_robot_enabled_ = get_parameter("clear_robot").as_bool();
  robot_height_ = get_parameter("robot_height").as_double();
  robot_radius_ = get_parameter("robot_radius").as_double();
  pub_rate_ = get_parameter("pub_rate").as_double();
  cloud_in_queue_size_ = get_parameter("cloud_in_queue_size").as_int();
  map_queue_size_ = get_parameter("map_queue_size").as_int();
  map_binary_queue_size_ = get_parameter("map_binary_queue_size").as_int();
  map_cloud_queue_size_ = get_parameter("map_cloud_queue_size").as_int();

  // Publishers
  rclcpp::QoS map_pub_qos(map_queue_size_);
  if (get_parameter("map_latch").as_bool()) {
    map_pub_qos.transient_local();
  }
  map_pub_ = create_publisher<ufomap_msgs::msg::Ufomap>("map", map_pub_qos);

  rclcpp::QoS map_binary_pub_qos(map_binary_queue_size_);
  if (get_parameter("map_binary_latch").as_bool()) {
    map_binary_pub_qos.transient_local();
  }
  map_binary_pub_ = create_publisher<ufomap_msgs::msg::Ufomap>("map_binary", map_binary_pub_qos);

  rclcpp::QoS map_cloud_pub_qos(map_cloud_queue_size_);
  if (get_parameter("map_cloud_latch").as_bool()) {
    map_cloud_pub_qos.transient_local();
  }
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", map_cloud_pub_qos);

  // Subscribers
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in",
    get_parameter("cloud_in_queue_size").as_int(),
    std::bind(&UFOMapServer::cloudCallback, this, _1));
  
  // TF2 Listener
  tf_buffer_ = std::make_shared<tf2::BufferCore>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  // UFOMap
  map_ = std::make_shared<ufomap::Octree>(
    get_parameter("resolution").as_double(),
    get_parameter("depth_levels").as_int(),
    get_parameter("multithreaded").as_bool());

  // Timer initialization
	if (0 < pub_rate_)
	{
    pub_timer_ = create_wall_timer(
      rclcpp::Rate(pub_rate_).period(), std::bind(&UFOMapServer::timerCallback, this));
	}
}

// Private functions
void UFOMapServer::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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
		 	return tf_buffer_->lookupTransform(frame_id_, msg->header.frame_id, tf2_ros::fromMsg(msg->header.stamp));
		// 																		rclcpp::Time(msg->header.stamp), transform_timeout_);
		});
                                      
		auto a2 =
				std::async(std::launch::async, [&msg, &cloud] { ufomap::toUfomap(msg, cloud); });

		ufomap_math::Pose6 transform = ufomap::toUfomap(a1.get().transform);
		a2.wait();
		cloud.transform(transform);
		if (insert_discrete_)
		{
			map_->insertPointCloudDiscrete(transform.translation(), cloud, max_range_, insert_n_,
																		insert_depth_);
		}
		else
		{
			map_->insertPointCloud(transform.translation(), cloud, max_range_);
		}

		if (clear_robot_enabled_)
		{
			ufomap::Point3 robot_bbx_min(transform.x() - robot_radius_,
																	 transform.y() - robot_radius_,
																	 transform.z() - (robot_height_ / 2.0));
			ufomap::Point3 robot_bbx_max(transform.x() + robot_radius_,
																	 transform.y() + robot_radius_,
																	 transform.z() + (robot_height_ / 2.0));
			map_->clearAreaBBX(robot_bbx_min, robot_bbx_max, 0);
		}
	}
	catch (tf2::TransformException& ex)
	{
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1, "%s", ex.what());
	}
}

void UFOMapServer::timerCallback()
{
	std_msgs::msg::Header header;
	header.stamp = now();
	header.frame_id = frame_id_;

	if (0 < map_pub_->get_subscription_count() ||
    map_pub_->get_actual_qos().get_rmw_qos_profile().durability ==
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
	{
		ufomap_msgs::msg::Ufomap msg;
		ufomap_msgs::mapToMsg(*map_, msg, false);
		msg.header = header;
		map_pub_->publish(msg);
	}

	if (0 < map_binary_pub_->get_subscription_count() ||
    map_binary_pub_->get_actual_qos().get_rmw_qos_profile().durability ==
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) 
  {
		ufomap_msgs::msg::Ufomap msg;
		ufomap_msgs::mapToMsg(*map_, msg, false, true);
		msg.header = header;
		map_binary_pub_->publish(msg);
	}

	if (0 < cloud_pub_->get_subscription_count() ||
    cloud_pub_->get_actual_qos().get_rmw_qos_profile().durability ==
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
	{
		ufomap::PointCloud cloud;
		for (auto it = map_->begin_leafs(true, false, false, false, 0),
							it_end = map_->end_leafs();
				 it != it_end; ++it)
		{
			cloud.push_back(it.getCenter());
		}
		sensor_msgs::msg::PointCloud2::Ptr cloud_msg(new sensor_msgs::msg::PointCloud2);
		ufomap::fromUfomap(cloud, cloud_msg);
		cloud_msg->header = header;
		cloud_pub_->publish(*cloud_msg);
	}
}

}  // namespace ufomap_mapping
