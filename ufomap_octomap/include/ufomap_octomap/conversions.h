#ifndef UFOMAP_OCTOMAP_CONVERSIONS_H
#define UFOMAP_OCTOMAP_CONVERSIONS_H

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <ufomap/ufomap.h>

#include <bitset>
#include <sstream>

namespace ufomap
{
template <typename OCTOMAP_TYPE, typename UFOMAP_TYPE>
void toUfomap(const octomap::OccupancyOcTreeBase<OCTOMAP_TYPE>& map_in,
							ufomap::OctreeBase<UFOMAP_TYPE>& map_out)
{
	unsigned int tree_depth = map_in.getTreeDepth();
	map_out.clear(map_in.getResolution(), tree_depth);
	bool add_color =
			"OctreeRGB" == map_out.getTreeType() && "ColorOcTree" == map_in.getTreeType();
	for (auto it = map_in.begin_leafs(), it_end = map_in.end_leafs(); it != it_end; ++it)
	{
		auto coord = it.getCoordinate();
		map_out.setNodeValue(coord.x(), coord.y(), coord.z(), it->getLogOdds(),
												 tree_depth - it.getDepth());
		if (add_color)
		{
			auto color = it->getColor();
			map_out.setNodeColor(coord.x(), coord.y(), coord.z(), color.r, color.g, color.b,
													 tree_depth - it.getDepth());
		}
	}
}

template <typename UFOMAP_TYPE, typename OCTOMAP_TYPE>
void fromUfomap(const ufomap::OctreeBase<UFOMAP_TYPE>& map_in,
								octomap::OccupancyOcTreeBase<OCTOMAP_TYPE>& map_out)
{
	// Clear OctoMap
	map_out.setResolution(map_in.getResolution());
	map_out.clear();

	bool add_color =
			"OctreeRGB" == map_in.getTreeType() && "ColorOcTree" == map_out.getTreeType();

	std::stringstream s(std::ios_base::in | std::ios_base::out | std::ios_base::binary);

	unsigned int min_depth =
			std::max(0, int(map_in.getTreeDepthLevels()) - int(map_out.getTreeDepth()));

	for (auto it = map_in.begin_tree(true, true, false, false, min_depth),
						it_end = map_in.end_tree();
			 it != it_end; ++it)
	{
		OCTOMAP_TYPE node;
		node.value = static_cast<double>(it.getLogit());
		if (add_color)
		{
			node.color.r = it->node->color.r;
			node.color.g = it->node->color.g;
			node.color.b = it->node->color.b;
		}
		node.writeData(s);

		// 1 bit for each children; 0 empty, 1: allocated
		std::bitset<8> children;
		if (it.getDepth() != min_depth && it.hasChildren())
		{
			for (unsigned int i = 0; i < 8; ++i)
			{
				if (!map_in.isUnknown(it->code.getChild(i)))
				{
					children[i] = 1;
				}
			}
		}

		char children_char = (char)children.to_ulong();
		s.write((char*)&children_char, sizeof(char));
	}

	// Construct the OctoMap
	map_out.readData(s);
}
}  // namespace ufomap

#endif  // UFOMAP_OCTOMAP_CONVERSIONS_H