#include <ufomap_octomap/conversions.h>

namespace ufomap
{
void toUfomap(const octomap::OcTree& map_in, ufomap::Octree& map_out)
{
	unsigned int tree_depth = map_in.getTreeDepth();
	map_out.clear(map_in.getResolution(), tree_depth);
	for (auto it = map_in.begin_leafs(), it_end = map_in.end_leafs(); it != it_end; ++it)
	{
		auto coord = it.getCoordinate();
		map_out.setNodeValue(coord.x(), coord.y(), coord.z(), it->getLogOdds(),
												 tree_depth - it.getDepth());
	}
}

void toUfomap(const octomap::ColorOcTree& map_in, ufomap::OctreeRGB& map_out)
{
	unsigned int tree_depth = map_in.getTreeDepth();
	map_out.clear(map_in.getResolution(), tree_depth);
	for (auto it = map_in.begin_leafs(), it_end = map_in.end_leafs(); it != it_end; ++it)
	{
		auto coord = it.getCoordinate();
		auto color = it->getColor();
		map_out.setNodeValue(coord.x(), coord.y(), coord.z(), it->getLogOdds(),
												 tree_depth - it.getDepth());
		map_out.setNodeColor(coord.x(), coord.y(), coord.z(), color.r, color.g, color.b,
												 tree_depth - it.getDepth());
	}
}

void fromUfomap(const ufomap::Octree& map_in, octomap::OcTree& map_out)
{
	map_out.setResolution(map_in.getResolution());
	map_out.clear();
	for (auto it = map_in.begin_leafs(), it_end = map_in.end_leafs(); it != it_end; ++it)
	{
		// TODO: Implement
	}
}

void fromUfomap(const ufomap::OctreeRGB& map_in, octomap::ColorOcTree& map_out)
{
	map_out.setResolution(map_in.getResolution());
	map_out.clear();
	for (auto it = map_in.begin_leafs(), it_end = map_in.end_leafs(); it != it_end; ++it)
	{
		// TODO: Implement
	}
}
}  // namespace ufomap