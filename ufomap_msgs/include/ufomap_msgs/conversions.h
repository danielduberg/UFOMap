#ifndef UFOMAP_MSGS_CONVERSIONS_H
#define UFOMAP_MSGS_CONVERSIONS_H

#include <ufomap/geometry/bounding_volume.h>
#include <ufomap_msgs/Ufomap.h>

#include <type_traits>

namespace ufomap_msgs
{
//
// ROS message type to UFOMap type
//

ufomap_math::Vector3 msgToUfomap(const ufomap_msgs::Point& point);

ufomap_geometry::AABB msgToUfomap(const ufomap_msgs::AABB& aabb);

ufomap_geometry::Plane msgToUfomap(const ufomap_msgs::Plane& plane);

ufomap_geometry::Frustum msgToUfomap(const ufomap_msgs::Frustum& frustum);

ufomap_geometry::LineSegment msgToUfomap(const ufomap_msgs::LineSegment& line_segment);

ufomap_geometry::OBB msgToUfomap(const ufomap_msgs::OBB& obb);

ufomap_geometry::Ray msgToUfomap(const ufomap_msgs::Ray& ray);

ufomap_geometry::Sphere msgToUfomap(const ufomap_msgs::Sphere& sphere);

ufomap_geometry::BoundingVolume msgToUfomap(const ufomap_msgs::BoundingVolume& msg);

template <typename TreeType>
bool msgToUfomap(const Ufomap& msg, TreeType& tree)
{
	std::stringstream data_stream(std::ios_base::in | std::ios_base::out |
																std::ios_base::binary);
	if (!msg.data.empty())
	{
		data_stream.write((const char*)&msg.data[0], msg.data.size());
		return tree.readData(
				data_stream, msgToUfomap(msg.info.bounding_volume), msg.info.resolution,
				msg.info.depth_levels, msg.info.occupancy_thres, msg.info.free_thres,
				msg.info.uncompressed_data_size, msg.info.compressed, msg.info.binary);
	}
	return false;
}

//
// UFOMap type to ROS message type
//

ufomap_msgs::Point ufomapToMsg(const ufomap_math::Vector3& point);

ufomap_msgs::AABB ufomapToMsg(const ufomap_geometry::AABB& aabb);

ufomap_msgs::Plane ufomapToMsg(const ufomap_geometry::Plane& plane);

ufomap_msgs::Frustum ufomapToMsg(const ufomap_geometry::Frustum& frustum);

ufomap_msgs::LineSegment ufomapToMsg(const ufomap_geometry::LineSegment& line_segment);

ufomap_msgs::OBB ufomapToMsg(const ufomap_geometry::OBB& obb);

ufomap_msgs::Ray ufomapToMsg(const ufomap_geometry::Ray& ray);

ufomap_msgs::Sphere ufomapToMsg(const ufomap_geometry::Sphere& sphere);

ufomap_msgs::BoundingVolume
ufomapToMsg(const ufomap_geometry::BoundingVolume& bounding_volume);

template <typename TreeType>
bool ufomapToMsg(const TreeType& tree, Ufomap& msg, bool compress = false,
								 bool binary = false, unsigned int depth = 0)
{
	return ufomapToMsg(tree, msg, ufomap_geometry::BoundingVolume(), compress, binary,
										 depth);
}

template <typename TreeType, typename BoundingType>
bool ufomapToMsg(const TreeType& tree, Ufomap& msg, const BoundingType& bounding_volume,
								 bool compress = false, bool binary = false, unsigned int depth = 0)
{
	ufomap_geometry::BoundingVolume bv;
	bv.add(bounding_volume);
	return ufomapToMsg(tree, msg, bv, compress, binary, depth);
}

template <typename TreeType>
bool ufomapToMsg(const TreeType& tree, Ufomap& msg,
								 const ufomap_geometry::BoundingVolume& bounding_volume,
								 bool compress = false, bool binary = false, unsigned int depth = 0)
{
	msg.info.version = tree.getFileVersion();
	msg.info.id = tree.getTreeType();
	msg.info.binary = binary;
	msg.info.resolution = tree.getResolution();
	msg.info.depth_levels = tree.getTreeDepthLevels();
	msg.info.occupancy_thres = tree.getOccupancyThres();
	msg.info.free_thres = tree.getFreeThres();
	msg.info.compressed = compress;
	msg.info.bounding_volume = ufomapToMsg(bounding_volume);

	std::stringstream data_stream(std::ios_base::in | std::ios_base::out |
																std::ios_base::binary);
	msg.info.uncompressed_data_size =
			tree.writeData(data_stream, bounding_volume, compress, binary, depth);
	if (0 > msg.info.uncompressed_data_size)
	{
		return false;
	}

	const std::string& data_string = data_stream.str();
	msg.data = std::vector<int8_t>(data_string.begin(), data_string.end());
	return true;
}
}  // namespace ufomap_msgs

#endif  // UFOMAP_MSGS_CONVERSIONS_H