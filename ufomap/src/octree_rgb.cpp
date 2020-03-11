#include <ufomap/octree_rgb.h>

#include <sstream>

namespace ufomap
{
//
// Constructors and destructors
//

OctreeRGB::OctreeRGB(float resolution, unsigned int depth_levels, bool automatic_pruning,
										 bool prune_consider_color, float occupancy_thres, float free_thres,
										 float prob_hit, float prob_miss, float clamping_thres_min,
										 float clamping_thres_max)
	: OctreeBase(resolution, depth_levels, automatic_pruning, occupancy_thres, free_thres,
							 prob_hit, prob_miss, clamping_thres_min, clamping_thres_max)
	, prune_consider_color_(prune_consider_color)
	, rng_(dev_())
	, dist_(0, 10)
{
}

OctreeRGB::OctreeRGB(const std::string& filename) : OctreeRGB()
{
	read(filename);
}

OctreeRGB::OctreeRGB(const OctreeRGB& other)
	: OctreeRGB(other.resolution_, other.depth_levels_, other.automatic_pruning_enabled_,
							other.prune_consider_color_, other.getOccupancyThres(),
							other.getFreeThres(), other.getProbHit(), other.getProbMiss(),
							other.getClampingThresMin(), other.getClampingThresMax())
{
	// TODO: Is correct?
	std::stringstream s(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
	other.write(s);
	read(s);
}

//
// Insertion
//

void OctreeRGB::insertPointCloud(const Point3& sensor_origin, const PointCloudRGB& cloud,
																 float max_range)
{
	PointCloud no_color_cloud;

	CodeMap<std::vector<Color>> colors;

	for (const Point3RGB& point : cloud)
	{
		if (0 > max_range || (point - sensor_origin).norm() < max_range)
		{
			colors[Code(coordToKey(point))].push_back(point.getColor());
			no_color_cloud.push_back(point);
		}
	}

	OctreeBase<OccupancyNodeRGB>::insertPointCloud(sensor_origin, no_color_cloud,
																								 max_range);

	for (const auto& [code, color] : colors)
	{
		for (const Color& c : color)
		{
			integrateColor(code, c);
		}
	}
}

void OctreeRGB::insertPointCloudDiscrete(const Point3& sensor_origin,
																				 const PointCloudRGB& cloud, float max_range,
																				 bool super_speed, unsigned int depth)
{
	PointCloud no_color_cloud;

	CodeMap<std::vector<Color>> colors;

	for (const Point3RGB& point : cloud)
	{
		if (0 > max_range || (point - sensor_origin).norm() < max_range)
		{
			std::vector<Color>& color = colors[Code(coordToKey(point))];
			if (color.empty())
			{
				no_color_cloud.push_back(point);
			}
			color.push_back(point.getColor());
		}
	}

	OctreeBase<OccupancyNodeRGB>::insertPointCloudDiscrete(sensor_origin, no_color_cloud,
																												 max_range, super_speed, depth);

	for (const auto& [code, color] : colors)
	{
		if (isOccupied(code))  // TODO: Does this improve performance?
		{
			// averageNodeColor(code, getAverageColor(color));
			integrateColor(code, getAverageColor(color));
		}
	}
}

//
// Set node color
//

Node<OccupancyNodeRGB> OctreeRGB::setNodeColor(const Code& code, Color color)
{
	Node<OccupancyNodeRGB> node = getNode(code, !prune_consider_color_);
	if (nullptr != node.node && isOccupied(node) && node.node->color != color)
	{
		setNodeColorRecurs(code, color, root_, depth_levels_);
	}
	return node;
}

//
// Average node color
//

Node<OccupancyNodeRGB> OctreeRGB::averageNodeColor(const Code& code, Color color)
{
	Node<OccupancyNodeRGB> node = getNode(code, !prune_consider_color_);
	if (nullptr != node.node && isOccupied(node))
	{
		Color color_not_set;
		if (node.node->color != color_not_set && node.node->color != color)
		{
			// TODO: Update to LAB space

			return setNodeColorRecurs(code, getAverageColor({ node.node->color, color }), root_,
																depth_levels_)
					.first;
		}
		else
		{
			return setNodeColorRecurs(code, color, root_, depth_levels_).first;
		}
	}
	return node;
}

//
// Integrate color
//

Node<OccupancyNodeRGB> OctreeRGB::integrateColor(const Code& code, Color color)
{
	Node<OccupancyNodeRGB> node = getNode(code, !prune_consider_color_);
	if (nullptr != node.node && isOccupied(node) && node.node->color != color)
	{
		Color color_not_set;
		if (node.node->color != color_not_set)
		{
			float node_prob = probability(node.node->logit);

			double r =
					std::sqrt((((double)(node.node->color.r * node.node->color.r)) * node_prob) +
										(((double)(color.r * color.r)) * (0.99 - node_prob)));
			double g =
					std::sqrt((((double)(node.node->color.g * node.node->color.g)) * node_prob) +
										(((double)(color.g * color.g)) * (0.99 - node_prob)));
			double b =
					std::sqrt((((double)(node.node->color.b * node.node->color.b)) * node_prob) +
										(((double)(color.b * color.b)) * (0.99 - node_prob)));

			node = setNodeColorRecurs(code, Color(r, g, b), root_, depth_levels_).first;
		}
		else
		{
			return setNodeColorRecurs(code, color, root_, depth_levels_).first;
		}
	}
	return node;
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Protected functions ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

//
// Set node color recurs
//

std::pair<Node<OccupancyNodeRGB>, bool>
OctreeRGB::setNodeColorRecurs(const Code& code, const Color& color,
															OccupancyNodeRGB& node, unsigned int current_depth)
{
	if (current_depth > code.getDepth())
	{
		InnerNode<OccupancyNodeRGB>& inner_node =
				static_cast<InnerNode<OccupancyNodeRGB>&>(node);

		if (!hasChildren(inner_node))
		{
			createChildren(inner_node, current_depth);
		}

		unsigned int child_depth = current_depth - 1;

		// Get child index
		unsigned int child_idx = code.getChildIdx(child_depth);

		// Get child
		OccupancyNodeRGB* child_node =
				(0 == child_depth) ? &(*static_cast<std::array<OccupancyNodeRGB, 8>*>(
																 inner_node.children))[child_idx] :
														 &(*static_cast<std::array<InnerNode<OccupancyNodeRGB>, 8>*>(
																 inner_node.children))[child_idx];

		// Recurs
		auto [child, changed] = setNodeColorRecurs(code, color, *child_node, child_depth);

		if (changed)
		{
			// Update this node
			changed = OctreeBase<OccupancyNodeRGB>::updateNode(inner_node, current_depth);
		}
		return std::make_pair(child, changed);
	}
	else
	{
		// Update color
		node.color = color;
		if (0 < current_depth)
		{
			InnerNode<OccupancyNodeRGB>& inner_node =
					static_cast<InnerNode<OccupancyNodeRGB>&>(node);

			if (hasChildren(inner_node))
			{
				unsigned int child_depth = current_depth - 1;
				for (unsigned int child_idx = 0; child_idx < 8; ++child_idx)
				{
					OccupancyNodeRGB* child_node =
							(0 == child_depth) ?
									&(*static_cast<std::array<OccupancyNodeRGB, 8>*>(
											inner_node.children))[child_idx] :
									&(*static_cast<std::array<InnerNode<OccupancyNodeRGB>, 8>*>(
											inner_node.children))[child_idx];
					if (isOccupied(*child_node))
					{
						setNodeColorRecurs(code.getChild(child_idx), color, *child_node, child_depth);
					}
				}
				// Update this node
				OctreeBase<OccupancyNodeRGB>::updateNode(inner_node, current_depth);
			}
		}
		return std::make_pair(Node<OccupancyNodeRGB>(&node, code), true);
	}
}

//
// Node collapsible
//

bool OctreeRGB::isNodeCollapsible(const std::array<OccupancyNodeRGB, 8>& children) const
{
	if (!prune_consider_color_ || !isOccupied(children[0]))
	{
		return OctreeBase<OccupancyNodeRGB>::isNodeCollapsible(children);
	}

	for (int i = 1; i < 8; ++i)
	{
		if (children[0].logit != children[i].logit || children[0].color != children[i].color)
		{
			return false;
		}
	}
	return true;
}

bool OctreeRGB::isNodeCollapsible(
		const std::array<InnerNode<OccupancyNodeRGB>, 8>& children) const
{
	if (!prune_consider_color_ || !isOccupied(children[0]))
	{
		return OctreeBase<OccupancyNodeRGB>::isNodeCollapsible(children);
	}

	if (isLeaf(children[0]))
	{
		for (int i = 1; i < 8; ++i)
		{
			if (children[0].logit != children[i].logit ||
					children[0].color != children[i].color || !isLeaf(children[i]))
			{
				return false;
			}
		}
	}
	else
	{
		return false;
	}

	return true;
}

//
// Update node
//

bool OctreeRGB::updateNode(InnerNode<OccupancyNodeRGB>& node,
													 const std::array<OccupancyNodeRGB, 8>& children,
													 unsigned int depth)
{
	Color new_color = getAverageChildColor(children);
	bool changed = OctreeBase<OccupancyNodeRGB>::updateNode(node, children, depth);
	changed = changed || (node.color != new_color);
	node.color = isOccupied(node) ? new_color : Color();
	return changed;
}

bool OctreeRGB::updateNode(InnerNode<OccupancyNodeRGB>& node,
													 const std::array<InnerNode<OccupancyNodeRGB>, 8>& children,
													 unsigned int depth)

{
	Color new_color = getAverageChildColor(children);
	bool changed = OctreeBase<OccupancyNodeRGB>::updateNode(node, children, depth);
	changed = changed || (node.color != new_color);
	node.color = isOccupied(node) ? new_color : Color();
	return changed;
}

//
// Average child color
//

Color OctreeRGB::getAverageChildColor(
		const std::array<OccupancyNodeRGB, 8>& children) const
{
	std::vector<Color> colors;
	for (const OccupancyNodeRGB& child : children)
	{
		if (isOccupied(child))
		{
			colors.push_back(child.color);
		}
	}
	if (colors.empty())
	{
		return Color();
	}
	return getAverageColor(colors);
}

Color OctreeRGB::getAverageChildColor(
		const std::array<InnerNode<OccupancyNodeRGB>, 8>& children) const
{
	std::vector<Color> colors;
	for (const InnerNode<OccupancyNodeRGB>& child : children)
	{
		if (isOccupied(child))
		{
			colors.push_back(child.color);
		}
	}
	if (colors.empty())
	{
		return Color();
	}
	return getAverageColor(colors);
}

Color OctreeRGB::getAverageColor(const std::vector<Color>& colors) const
{
	// TODO: Update to LAB space?
	double r = 0;
	double g = 0;
	double b = 0;
	for (const Color& color : colors)
	{
		r += (color.r * color.r);
		g += (color.g * color.g);
		b += (color.b * color.b);
	}
	return Color(std::sqrt(r / colors.size()), std::sqrt(g / colors.size()),
							 std::sqrt(b / colors.size()));
	// for (const Color& color : colors)
	// {
	// 	r += color.r;
	// 	g += color.g;
	// 	b += color.b;
	// }
	// return Color(r / colors.size(), g / colors.size(), b / colors.size());
}

}  // namespace ufomap
