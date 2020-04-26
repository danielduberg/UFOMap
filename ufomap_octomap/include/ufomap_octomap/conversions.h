#ifndef UFOMAP_OCTOMAP_CONVERSIONS_H
#define UFOMAP_OCTOMAP_CONVERSIONS_H

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <ufomap/ufomap.h>

#include <bitset>
#include <sstream>
#include <type_traits>

namespace ufomap
{
template <typename OCTOMAP_TYPE, typename UFOMAP_TYPE>
void toUfomap(const octomap::OccupancyOcTreeBase<OCTOMAP_TYPE>& map_in,
							ufomap::OctreeBase<UFOMAP_TYPE>& map_out)
{
	unsigned int tree_depth = map_in.getTreeDepth();
	map_out.clear(map_in.getResolution(), tree_depth);
	const bool add_color = std::is_same<UFOMAP_TYPE, ufomap::OctreeRGB>::value &&
												 std::is_same<OCTOMAP_TYPE, octomap::ColorOcTree>::value;
	for (auto it = map_in.begin_leafs(), it_end = map_in.end_leafs(); it != it_end; ++it)
	{
		auto coord = it.getCoordinate();
		Code code(
				map_out.coordToKey(coord.x(), coord.y(), coord.z(), tree_depth - it.getDepth()));
		map_out.setNodeValue(code, it->getLogOdds());
		if constexpr (add_color)
		{
			auto color = it->getColor();
			map_out.setNodeColor(code, color.r, color.g, color.b);
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

	const bool add_color = std::is_same<UFOMAP_TYPE, ufomap::OctreeRGB>::value &&
												 std::is_same<OCTOMAP_TYPE, octomap::ColorOcTree>::value;

	std::stringstream s(std::ios_base::in | std::ios_base::out | std::ios_base::binary);

	unsigned int min_depth =
			std::max(0, int(map_in.getTreeDepthLevels()) - int(map_out.getTreeDepth()));

	OCTOMAP_TYPE node;
	for (auto it = map_in.begin_tree(true, true, false, true, min_depth),
						it_end = map_in.end_tree();
			 it != it_end; ++it)
	{
		node.setValue(static_cast<double>(it.getLogit()));
		if constexpr (add_color)
		{
			node.setColor(it->node->color.r, it->node->color.g, it->node->color.b);
		}
		node.writeData(s);

		// 1 bit for each children; 0 empty, 1: allocated
		std::bitset<8> children;
		if (min_depth != it.getDepth() && it.hasChildren())
		{
			for (unsigned int i = 0; i < 8; ++i)
			{
				Code child_code = it->code.getChild(i);
				if (map_in.containsFree(child_code) || map_in.containsOccupied(child_code))
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
	// Need to call this because otherwise some inner nodes will have a value of 0.5
	// (unknown) when in reality it should be free. This is because in UFOMap a node has
	// either 0 or 8 children and unknown nodes have a higher value than free nodes.
	map_out.updateInnerOccupancy();
}
}  // namespace ufomap

#endif  // UFOMAP_OCTOMAP_CONVERSIONS_H