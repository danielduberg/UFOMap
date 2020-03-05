#ifndef UFOMAP_MSGS_CONVERSIONS_H
#define UFOMAP_MSGS_CONVERSIONS_H

#include <ufomap_msgs/Ufomap.h>

namespace ufomap_msgs
{
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

template <typename TreeType>
bool mapToMsgData(const TreeType& tree, std::vector<int8_t>& map_data,
									bool binary = false)
{
	std::stringstream data_stream;
	if (!tree.writeData(data_stream, binary))
	{
		return false;
	}

	std::string data_string = data_stream.str();
	map_data = std::vector<int8_t>(data_string.begin(), data_string.end());
	return true;
}

template <typename TreeType>
bool mapToMsg(const TreeType& tree, Ufomap& msg, bool binary = false)
{
	msg.resolution = tree.getResolution();
	msg.id = tree.getTreeType();
	msg.depth_levels = tree.getTreeDepthLevels();
	msg.binary = binary;

	return mapToMsgData(tree, msg.data);
}
}  // namespace ufomap_msgs

#endif  // UFOMAP_MSGS_CONVERSIONS_H