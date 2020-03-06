#ifndef UFOMAP_NODE_H
#define UFOMAP_NODE_H

#include <ufomap/code.h>
#include <ufomap/color.h>

#include <array>
#include <iostream>

// TODO: Update documentation

namespace ufomap
{
/**
 * @brief An occupancy leaf node
 *
 */
struct OccupancyNode
{
	// The occupancy value of the node
	float logit = 0.0;

	/**
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @param is_occupied Whether this node is consider occupied
	 * @param to_octomap Whether to write data in a UFOMap format or OctoMap format
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s, float occupancy_thres_log,
													float free_thres_log, bool to_octomap = false) const
	{
		if (to_octomap)
		{
			double value = logit;
			s.write((const char*)&value, sizeof(value));
		}
		else
		{
			s.write((const char*)&logit, sizeof(logit));
		}
		return s;
	}

	/**
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @param is_occupied Whether this node is consider occupied
	 * @param from_octomap Whether the data in the stream is in UFOMap format or OctoMap
	 * format
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s, float occupancy_thres_log, float free_thres_log,
												 bool from_octomap = false)
	{
		if (from_octomap)
		{
			double value;
			s.read((char*)&value, sizeof(value));
			logit = value;
		}
		else
		{
			s.read((char*)&logit, sizeof(logit));
		}
		return s;
	}
};

/**
 * @brief An occupancy RGB leaf node
 *
 */
struct OccupancyNodeRGB : OccupancyNode
{
	// The RGB color of the node
	Color color;

	/**
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @param is_occupied Whether this node is consider occupied
	 * @param to_octomap Whether to write data in a UFOMap format or OctoMap format
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s, float occupancy_thres_log,
													float free_thres_log, bool to_octomap = false) const
	{
		OccupancyNode::writeData(s, occupancy_thres_log, free_thres_log, to_octomap);
		if (logit > occupancy_thres_log || to_octomap)
		{
			s.write((const char*)&color, sizeof(color));
		}
		return s;
	}

	/**
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @param is_occupied Whether this node is consider occupied
	 * @param from_octomap Whether the data in the stream is in UFOMap format or OctoMap
	 * format
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s, float occupancy_thres_log, float free_thres_log,
												 bool from_octomap = false)
	{
		OccupancyNode::readData(s, occupancy_thres_log, free_thres_log, from_octomap);
		if (logit > occupancy_thres_log || from_octomap)
		{
			s.read((char*)&color, sizeof(color));
		}
		return s;
	}
};

/**
 * @brief An occupancy intensity leaf node
 *
 */
struct OccupancyNodeIntensity : OccupancyNode
{
	// The intensity of the node
	uint8_t intensity;

	/**
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @param is_occupied Whether this node is consider occupied
	 * @param to_octomap Whether to write data in a UFOMap format or OctoMap format
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s, float occupancy_thres_log,
													float free_thres_log, bool to_octomap = false) const
	{
		OccupancyNode::writeData(s, occupancy_thres_log, free_thres_log, to_octomap);
		if (logit > occupancy_thres_log || to_octomap)
		{
			s.write((const char*)&intensity, sizeof(intensity));
		}
		return s;
	}

	/**
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @param is_occupied Whether this node is consider occupied
	 * @param from_octomap Whether the data in the stream is in UFOMap format or OctoMap
	 * format
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s, float occupancy_thres_log, float free_thres_log,
												 bool from_octomap = false)
	{
		OccupancyNode::readData(s, occupancy_thres_log, free_thres_log, from_octomap);
		if (logit > occupancy_thres_log || from_octomap)
		{
			s.read((char*)&intensity, sizeof(intensity));
		}
		return s;
	}
};

/**
 * @brief An inner node for the octree
 *
 * @tparam DATA_TYPE The type of data this node should store
 */
template <typename DATA_TYPE>
struct InnerNode : DATA_TYPE
{
	// Important that the bools are before the children because of alignment on 64 bit

	// Indicates whether this node or any of its children contains free space
	bool contains_free = false;
	// Indicates whether this node or any of its children contains unknown space
	bool contains_unknown = true;
	// Indicates whether all of this children are the same. If this is true, there is no
	// reason to visit its children
	bool all_children_same = true;

	/**
	 * @brief A pointer to the children of this node.
	 *
	 * @remark You have to delete the children yourself
	 */
	void* children = nullptr;
};

/**
 * @brief
 *
 * @tparam LEAF_NODE
 */
template <typename LEAF_NODE>
struct Node
{
	const LEAF_NODE* node;  // A pointer to the actual node
	Code code;              // The code for this node. Used to find it in the octree

	Node()
	{
	}

	Node(const LEAF_NODE* node, const Code& code) : node(node), code(code)
	{
	}

	Node(const Node& other) : node(other.node), code(other.code)
	{
	}

	Node& operator=(const Node& rhs)
	{
		node = rhs.node;
		code = rhs.code;
		return *this;
	}

	bool operator==(const Node& rhs) const
	{
		return node == rhs.node && code == rhs.code;
	}

	bool operator!=(const Node& rhs) const
	{
		return node != rhs.node || code != rhs.code;
	}

	/**
	 * @brief Get the depth of this node
	 *
	 * @return unsigned int The depth of the node
	 */
	unsigned int getDepth() const
	{
		return code.getDepth();
	}
};

}  // namespace ufomap

#endif  // UFOMAP_NODE_H