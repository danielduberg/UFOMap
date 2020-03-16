#include <ufomap/octree.h>

#include <sstream>

namespace ufomap
{
Octree::Octree(float resolution, unsigned int depth_levels, bool automatic_pruning,
							 float occupancy_thres, float free_thres, float prob_hit, float prob_miss,
							 float clamping_thres_min, float clamping_thres_max)
	: OctreeBase(resolution, depth_levels, automatic_pruning, occupancy_thres, free_thres,
							 prob_hit, prob_miss, clamping_thres_min, clamping_thres_max)
{
}

Octree::Octree(const std::string& filename) : Octree()
{
	read(filename);
}

Octree::Octree(const Octree& other)
	: Octree(other.resolution_, other.depth_levels_, other.automatic_pruning_enabled_,
					 other.getOccupancyThres(), other.getFreeThres(), other.getProbHit(),
					 other.getProbMiss(), other.getClampingThresMin(), other.getClampingThresMax())
{
	// TODO: Is correct?
	std::stringstream s(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
	other.write(s);
	read(s);
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// PROTECTED ///////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

//
// Read/write
//

bool Octree::readBinaryNodesRecurs(std::istream& s, InnerNode<OccupancyNode>& node,
																	 unsigned int current_depth, float occupancy_thres_log,
																	 float free_thres_log, bool from_octomap)
{
	std::bitset<8> children_data_1;
	std::bitset<8> children_data_2;

	if (from_octomap)
	{
		char child1to4_char;
		char child5to8_char;
		s.read((char*)&child1to4_char, sizeof(char));
		s.read((char*)&child5to8_char, sizeof(char));

		std::bitset<8> child1to4((unsigned long long)child1to4_char);
		std::bitset<8> child5to8((unsigned long long)child5to8_char);
		for (unsigned int i = 0; i < 4; ++i)
		{
			children_data_1[i] = child1to4[i * 2];
			children_data_1[i + 4] = child5to8[i * 2];
			children_data_2[i] = child1to4[i * 2 + 1];
			children_data_2[i + 4] = child5to8[i * 2 + 1];
		}
	}
	else
	{
		char children_data_1_char;
		char children_data_2_char;
		s.read((char*)&children_data_1_char, sizeof(char));
		s.read((char*)&children_data_2_char, sizeof(char));
		children_data_1 = std::bitset<8>((unsigned long long)children_data_1_char);
		children_data_2 = std::bitset<8>((unsigned long long)children_data_2_char);
	}

	node.logit = clamping_thres_max_log_;

	if (children_data_1.any() || children_data_2.any())
	{
		createChildren(node, current_depth);

		if (1 == current_depth)
		{
			for (unsigned int i = 0; i < 8; ++i)
			{
				if (1 == children_data_1[i] && 0 == children_data_2[i])
				{
					// Free leaf
					(*static_cast<std::array<OccupancyNode, 8>*>(node.children))[i].logit =
							clamping_thres_min_log_;
				}
				else if (0 == children_data_1[i] && 1 == children_data_2[i])
				{
					// Occupied leaf
					(*static_cast<std::array<OccupancyNode, 8>*>(node.children))[i].logit =
							clamping_thres_max_log_;
				}
				else
				{
					// Unknown leaf
					(*static_cast<std::array<OccupancyNode, 8>*>(node.children))[i].logit =
							(occupancy_thres_log_ + free_thres_log_) / 2.0;
				}
			}
		}
		else
		{
			for (unsigned int i = 0; i < 8; ++i)
			{
				InnerNode<OccupancyNode>& child =
						(*static_cast<std::array<InnerNode<OccupancyNode>, 8>*>(node.children))[i];
				if (1 == children_data_1[i] && 0 == children_data_2[i])
				{
					// Free inner leaf
					child.logit = clamping_thres_min_log_;
					child.contains_free = true;
					child.contains_unknown = false;
					child.all_children_same = true;
				}
				else if (0 == children_data_1[i] && 1 == children_data_2[i])
				{
					// Occupied inner leaf
					child.logit = clamping_thres_max_log_;
					child.contains_free = false;
					child.contains_unknown = false;
					child.all_children_same = true;
				}
				else if (1 == children_data_1[i] && 1 == children_data_2[i])
				{
					// Has children
					readBinaryNodesRecurs(s, child, current_depth - 1, occupancy_thres_log,
																free_thres_log, from_octomap);
				}
				else
				{
					// Unknown inner leaf
					child.logit = (occupancy_thres_log_ + free_thres_log_) / 2.0;
					child.contains_free = false;
					child.contains_unknown = true;
					child.all_children_same = true;
				}
			}
		}
	}

	updateNode(node, current_depth);

	return true;
}

bool Octree::writeBinaryNodesRecurs(std::ostream& s, const InnerNode<OccupancyNode>& node,
																		unsigned int current_depth, bool to_octomap) const
{
	std::bitset<8> children_data_1;
	std::bitset<8> children_data_2;

	if (hasChildren(node))
	{
		for (unsigned int i = 0; i < 8; ++i)
		{
			if (1 == current_depth)
			{
				const OccupancyNode& child =
						(*static_cast<std::array<OccupancyNode, 8>*>(node.children))[i];
				if (isOccupiedLog(child.logit))
				{
					children_data_1[i] = 0;
					children_data_2[i] = 1;
				}
				else if (isFreeLog(child.logit))
				{
					children_data_1[i] = 1;
					children_data_2[i] = 0;
				}
			}
			else
			{
				const InnerNode<OccupancyNode>& child =
						(*static_cast<std::array<InnerNode<OccupancyNode>, 8>*>(node.children))[i];
				if (hasChildren(child) && !containsOnlySameType(child))
				{
					children_data_1[i] = 1;
					children_data_2[i] = 1;
				}
				else if (isOccupiedLog(child.logit))
				{
					children_data_1[i] = 0;
					children_data_2[i] = 1;
				}
				else if (isFreeLog(child.logit))
				{
					children_data_1[i] = 1;
					children_data_2[i] = 0;
				}
			}
		}
	}

	if (to_octomap)
	{
		std::bitset<8> temp_1 = children_data_1;
		std::bitset<8> temp_2 = children_data_2;
		for (unsigned int i = 0; i < 4; ++i)
		{
			children_data_1[i * 2] = temp_1[i];
			children_data_2[i * 2] = temp_1[i + 4];
			children_data_1[i * 2 + 1] = temp_2[i];
			children_data_2[i * 2 + 1] = temp_2[i + 4];
		}
	}

	char children_data_1_char = (char)children_data_1.to_ulong();
	char children_data_2_char = (char)children_data_2.to_ulong();

	s.write((char*)&children_data_1_char, sizeof(char));
	s.write((char*)&children_data_2_char, sizeof(char));

	if (hasChildren(node) && 1 < current_depth)
	{
		for (const auto& child :
				 *static_cast<std::array<InnerNode<OccupancyNode>, 8>*>(node.children))
		{
			if (hasChildren(child) && !containsOnlySameType(child))
			{
				writeBinaryNodesRecurs(s, child, current_depth - 1, to_octomap);
			}
		}
	}

	return true;
}
}  // namespace ufomap