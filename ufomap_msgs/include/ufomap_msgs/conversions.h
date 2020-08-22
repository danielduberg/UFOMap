#ifndef UFOMAP_MSGS_CONVERSIONS_H
#define UFOMAP_MSGS_CONVERSIONS_H

#include "ufomap_msgs/msg/ufomap.h"

namespace ufomap_msgs
{
template <typename TreeType>
bool msgToMap(const msg::Ufomap& msg, TreeType& tree)
{
	std::stringstream data_stream(std::ios_base::in | std::ios_base::out |
																std::ios_base::binary);
	if (!msg.data.empty())
	{
		data_stream.write((const char*)&msg.data[0], msg.data.size());
		if (msg.compressed)
		{
			return tree.readDataCompressed(data_stream, msg.resolution, msg.depth_levels,
																		 msg.occupancy_thres, msg.free_thres, msg.data_size,
																		 msg.compressed_data_size, msg.binary);
		}
		else
		{
			return tree.readData(data_stream, msg.resolution, msg.depth_levels,
													 msg.occupancy_thres, msg.free_thres, msg.binary);
		}
	}
	return false;
}

template <typename TreeType>
bool mapToMsg(const TreeType& tree, msg::Ufomap& msg, bool compress = false,
							bool binary = false)
{
	msg.binary = binary;
	msg.id = tree.getTreeType();
	msg.resolution = tree.getResolution();
	msg.depth_levels = tree.getTreeDepthLevels();
	msg.occupancy_thres = tree.getOccupancyThres();
	msg.free_thres = tree.getFreeThres();
	msg.compressed = compress;

	std::stringstream data_stream(std::ios_base::in | std::ios_base::out |
																std::ios_base::binary);
	if (compress)
	{
		if (!tree.writeDataCompress(data_stream, msg.data_size, msg.compressed_data_size,
																binary))
		{
			return false;
		}
	}
	else
	{
		if (!tree.writeData(data_stream, binary))
		{
			return false;
		}
		msg.data_size = 0;
		msg.compressed_data_size = 0;
	}

	const std::string& data_string = data_stream.str();
	msg.data = std::vector<int8_t>(data_string.begin(), data_string.end());
	return true;
}
}  // namespace ufomap_msgs

#endif  // UFOMAP_MSGS_CONVERSIONS_H