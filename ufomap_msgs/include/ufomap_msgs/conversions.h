#ifndef UFOMAP_MSGS_CONVERSIONS_H
#define UFOMAP_MSGS_CONVERSIONS_H

#include <ufomap/geometry/bounding_volume.h>
#include <ufomap_msgs/Ufomap.h>

namespace ufomap_msgs
{
template <typename TreeType>
bool msgToMap(const Ufomap& msg, TreeType& tree)
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
}

template <typename TreeType>
bool mapToMsg(const TreeType& tree, Ufomap& msg, bool compress = false,
							bool binary = false, unsigned int depth = 0)
{
	ufomap_geometry::BoundingVolume bv;
	return mapToMsg(tree, msg, bv, compress, binary, depth);
}

template <typename TreeType, typename BoundingType>
bool mapToMsg(const TreeType& tree, Ufomap& msg, const BoundingType& bounding_volume,
							bool compress = false, bool binary = false, unsigned int depth = 0)
{
	ufomap_geometry::BoundingVolume bv;
	bv.add(bounding_volume);
	return mapToMsg(tree, msg, bv, compress, binary, depth);
}

template <typename TreeType>
bool mapToMsg(const TreeType& tree, Ufomap& msg,
							const ufomap_geometry::BoundingVolume& bounding_volume,
							bool compress = false, bool binary = false, unsigned int depth = 0)
{
	msg.binary = binary;
	msg.id = tree.getTreeType();
	msg.resolution = tree.getResolution();
	msg.depth_levels = tree.getTreeDepthLevels();
	msg.occupancy_thres = tree.getOccupancyThres();
	msg.free_thres = tree.getFreeThres();
	msg.compressed = compress;
	// TODO: Fill in bounding volume

	std::stringstream data_stream(std::ios_base::in | std::ios_base::out |
																std::ios_base::binary);
	auto [success, data_size] =
			tree.writeData(data_stream, bounding_volume, compress, binary, depth);
	if (!success)
	{
		return false;
	}

	msg.data_size = data_size;

	const std::string& data_string = data_stream.str();
	msg.data = std::vector<int8_t>(data_string.begin(), data_string.end());
	return true;
}
}  // namespace ufomap_msgs

#endif  // UFOMAP_MSGS_CONVERSIONS_H