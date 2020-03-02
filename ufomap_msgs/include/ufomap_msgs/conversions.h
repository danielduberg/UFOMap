#ifndef UFOMAP_MSGS_CONVERSIONS_H
#define UFOMAP_MSGS_CONVERSIONS_H

#include <ufomap_msgs/Ufomap.h>

namespace ufomap_msgs
{
/**
 * @brief Converts a ROS msg to UFOMap
 *
 * @code{.cpp}
	void mapCallback(const ufomap_msgs::Ufomap::ConstPtr& msg)
	{
		ufomap::Octree map(0.1);
		ufomap_msgs::msgToMap(*msg, map);
	}
 * @endcode
 *
 * @tparam TreeType The UFOMap tree type
 * @param msg The ROS msg
 * @param tree The UFOMap tree
 * @return true If the conversion was successful
 * @return false If the conversion failed
 */
template <typename TreeType>
bool msgToMap(const Ufomap& msg, TreeType& tree)
{
	std::stringstream data_stream;
	if (!msg.data.empty())
	{
		data_stream.write((const char*)&msg.data[0], msg.data.size());
		return tree.readData(data_stream, msg.resolution, msg.depth_levels, msg.binary);
	}
}

/**
 * @brief Converts UFOMap to a ROS msg
 *
 * @code{.cpp}
	ufomap::Octree map(0.1);
	ufomap_msgs::Ufomap msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	ufomap_msgs::mapToMsg(map, msg);
 * @endcode
 * @tparam TreeType The UFOMap tree type
 * @param tree The UFOMap tree
 * @param msg The ROS msg
 * @param compress If the msg should be compressed
 * @param binary If the UFOMap should be converted to binary (min/max occupancy value)
 * @return true If the conversion was successful
 * @return false If the conversion failed
 */
template <typename TreeType>
bool mapToMsg(const TreeType& tree, Ufomap& msg, bool compress = false,
							bool binary = false)
{
	msg.resolution = tree.getResolution();
	msg.id = tree.getTreeType();
	msg.depth_levels = tree.getTreeDepthLevels();
	msg.binary = binary;

	std::stringstream data_stream;
	if (!tree.writeData(data_stream, binary))
	{
		return false;
	}

	std::string data_string = data_stream.str();
	msg.data = std::vector<int8_t>(data_string.begin(), data_string.end());
	return true;
}
}  // namespace ufomap_msgs

#endif  // UFOMAP_MSGS_CONVERSIONS_H