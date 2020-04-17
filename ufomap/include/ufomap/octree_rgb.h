#ifndef UFOMAP_OCTREE_RGB_H
#define UFOMAP_OCTREE_RGB_H

#include <ufomap/node.h>
#include <ufomap/octree_base.h>
#include <ufomap/types.h>

#include <random>

namespace ufomap
{
class OctreeRGB : public OctreeBase<OccupancyNodeRGB>
{
public:
	//
	// Constructors and destructors
	//

	/**
	 * @brief Default constructor
	 *
	 * @param resolution
	 * @param depth_levels
	 * @param automatic_pruning
	 * @param occupancy_thres
	 * @param free_thres
	 * @param prob_hit
	 * @param prob_miss
	 * @param clamping_thres_min
	 * @param clamping_thres_max
	 */
	OctreeRGB(float resolution = 0.1, unsigned int depth_levels = 16,
						bool automatic_pruning = true, bool prune_consider_color = false,
						float occupancy_thres = 0.5, float free_thres = 0.5, float prob_hit = 0.7,
						float prob_miss = 0.4, float clamping_thres_min = 0.1192,
						float clamping_thres_max = 0.971);

	OctreeRGB(const std::string& filename);

	/**
	 * @brief Copy constructor
	 *
	 * @param other
	 */
	OctreeRGB(const OctreeRGB& other);

	/**
	 * @brief Destructor
	 *
	 */
	virtual ~OctreeRGB()
	{
	}

	//
	// Tree type
	//

	virtual std::string getTreeType() const
	{
		return "OctreeRGB";
	}

	//
	// Insertion
	//

	void insertPointCloud(const Point3& sensor_origin, const PointCloudRGB& cloud,
												float max_range = -1);

	void insertPointCloudDiscrete(const Point3& sensor_origin, const PointCloudRGB& cloud,
																float max_range = -1, unsigned int n = 0,
																unsigned int depth = 0);

	void insertPointCloud(const Point3& sensor_origin, const PointCloudRGB& cloud,
												const Pose6& frame_origin, float max_range = -1)
	{
		PointCloudRGB cloud_transformed(cloud);
		cloud_transformed.transform(frame_origin);
		insertPointCloud(sensor_origin, cloud_transformed, max_range);
	}

	void insertPointCloudDiscrete(const Point3& sensor_origin, const PointCloudRGB& cloud,
																const Pose6& frame_origin, float max_range = -1,
																unsigned int n = 0, unsigned int depth = 0)
	{
		PointCloudRGB cloud_transformed(cloud);
		cloud_transformed.transform(frame_origin);
		insertPointCloudDiscrete(sensor_origin, cloud_transformed, max_range, n,
														 depth);
	}

	//
	// Set node color
	//

	Node<OccupancyNodeRGB> setNodeColor(const Node<OccupancyNodeRGB>& node, Color color)
	{
		return setNodeColor(node.code, color);
	}

	Node<OccupancyNodeRGB> setNodeColor(const Code& code, Color color);

	Node<OccupancyNodeRGB> setNodeColor(const Key& key, Color color)
	{
		return setNodeColor(Code(key), color);
	}

	Node<OccupancyNodeRGB> setNodeColor(const Point3& coord, Color color,
																			unsigned int depth = 0)
	{
		return setNodeColor(coordToKey(coord, depth), color);
	}

	Node<OccupancyNodeRGB> setNodeColor(float x, float y, float z, Color color,
																			unsigned int depth = 0)
	{
		return setNodeColor(coordToKey(x, y, z, depth), color);
	}

	Node<OccupancyNodeRGB> setNodeColor(const Node<OccupancyNodeRGB>& node, uint8_t r,
																			uint8_t g, uint8_t b)
	{
		return setNodeColor(node, Color(r, g, b));
	}

	Node<OccupancyNodeRGB> setNodeColor(const Code& code, uint8_t r, uint8_t g, uint8_t b)
	{
		return setNodeColor(code, Color(r, g, b));
	}

	Node<OccupancyNodeRGB> setNodeColor(const Key& key, uint8_t r, uint8_t g, uint8_t b)
	{
		return setNodeColor(key, Color(r, g, b));
	}

	Node<OccupancyNodeRGB> setNodeColor(const Point3& coord, uint8_t r, uint8_t g,
																			uint8_t b, unsigned int depth = 0)
	{
		return setNodeColor(coord, Color(r, g, b), depth);
	}

	Node<OccupancyNodeRGB> setNodeColor(float x, float y, float z, uint8_t r, uint8_t g,
																			uint8_t b, unsigned int depth = 0)
	{
		return setNodeColor(x, y, z, Color(r, g, b), depth);
	}

	//
	// Average node color
	//

	Node<OccupancyNodeRGB> averageNodeColor(const Node<OccupancyNodeRGB>& node, Color color)
	{
		return averageNodeColor(node.code, color);
	}

	Node<OccupancyNodeRGB> averageNodeColor(const Code& code, Color color);

	Node<OccupancyNodeRGB> averageNodeColor(const Key& key, Color color)
	{
		return averageNodeColor(Code(key), color);
	}

	Node<OccupancyNodeRGB> averageNodeColor(const Point3& coord, Color color,
																					unsigned int depth = 0)
	{
		return averageNodeColor(coordToKey(coord, depth), color);
	}

	Node<OccupancyNodeRGB> averageNodeColor(float x, float y, float z, Color color,
																					unsigned int depth = 0)
	{
		return averageNodeColor(coordToKey(x, y, z, depth), color);
	}

	Node<OccupancyNodeRGB> averageNodeColor(const Node<OccupancyNodeRGB>& node, uint8_t r,
																					uint8_t g, uint8_t b)
	{
		return averageNodeColor(node, Color(r, g, b));
	}

	Node<OccupancyNodeRGB> averageNodeColor(const Code& code, uint8_t r, uint8_t g,
																					uint8_t b)
	{
		return averageNodeColor(code, Color(r, g, b));
	}

	Node<OccupancyNodeRGB> averageNodeColor(const Key& key, uint8_t r, uint8_t g, uint8_t b)
	{
		return averageNodeColor(key, Color(r, g, b));
	}

	Node<OccupancyNodeRGB> averageNodeColor(const Point3& coord, uint8_t r, uint8_t g,
																					uint8_t b, unsigned int depth = 0)
	{
		return averageNodeColor(coord, Color(r, g, b), depth);
	}

	Node<OccupancyNodeRGB> averageNodeColor(float x, float y, float z, uint8_t r, uint8_t g,
																					uint8_t b, unsigned int depth = 0)
	{
		return averageNodeColor(x, y, z, Color(r, g, b), depth);
	}

	//
	// Integrate color
	//

	Node<OccupancyNodeRGB> integrateColor(const Node<OccupancyNodeRGB>& node, Color color)
	{
		return integrateColor(node.code, color);
	}

	Node<OccupancyNodeRGB> integrateColor(const Code& code, Color color);

	Node<OccupancyNodeRGB> integrateColor(const Key& key, Color color)
	{
		return integrateColor(Code(key), color);
	}

	Node<OccupancyNodeRGB> integrateColor(const Point3& coord, Color color,
																				unsigned int depth = 0)
	{
		return integrateColor(coordToKey(coord, depth), color);
	}

	Node<OccupancyNodeRGB> integrateColor(float x, float y, float z, Color color,
																				unsigned int depth = 0)
	{
		return integrateColor(coordToKey(x, y, z, depth), color);
	}

	Node<OccupancyNodeRGB> integrateColor(const Node<OccupancyNodeRGB>& node, uint8_t r,
																				uint8_t g, uint8_t b)
	{
		return integrateColor(node, Color(r, g, b));
	}

	Node<OccupancyNodeRGB> integrateColor(const Code& code, uint8_t r, uint8_t g, uint8_t b)
	{
		return integrateColor(code, Color(r, g, b));
	}

	Node<OccupancyNodeRGB> integrateColor(const Key& key, uint8_t r, uint8_t g, uint8_t b)
	{
		return integrateColor(key, Color(r, g, b));
	}

	Node<OccupancyNodeRGB> integrateColor(const Point3& coord, uint8_t r, uint8_t g,
																				uint8_t b, unsigned int depth = 0)
	{
		return integrateColor(coord, Color(r, g, b), depth);
	}

	Node<OccupancyNodeRGB> integrateColor(float x, float y, float z, uint8_t r, uint8_t g,
																				uint8_t b, unsigned int depth = 0)
	{
		return integrateColor(x, y, z, Color(r, g, b), depth);
	}

protected:
	//
	// Set node color recurs
	//

	std::pair<Node<OccupancyNodeRGB>, bool> setNodeColorRecurs(const Code& code,
																														 const Color& color,
																														 OccupancyNodeRGB& node,
																														 unsigned int current_depth);

	//
	// Node collapsible
	//

	virtual bool
	isNodeCollapsible(const std::array<OccupancyNodeRGB, 8>& children) const override;

	virtual bool isNodeCollapsible(
			const std::array<InnerNode<OccupancyNodeRGB>, 8>& children) const override;

	//
	// Update node
	//

	virtual bool updateNode(InnerNode<OccupancyNodeRGB>& node,
													const std::array<OccupancyNodeRGB, 8>& children,
													unsigned int depth) override;

	virtual bool updateNode(InnerNode<OccupancyNodeRGB>& node,
													const std::array<InnerNode<OccupancyNodeRGB>, 8>& children,
													unsigned int depth) override;

	//
	// Average child color
	//

	Color getAverageChildColor(const std::array<OccupancyNodeRGB, 8>& children) const;

	Color
	getAverageChildColor(const std::array<InnerNode<OccupancyNodeRGB>, 8>& children) const;

	Color getAverageColor(const std::vector<Color>& colors) const;

	//
	// Read/write
	//

	virtual bool binarySupport() const override
	{
		return false;
	}

protected:
	bool prune_consider_color_ = false;

	std::random_device dev_;
	std::mt19937 rng_;
	std::uniform_int_distribution<std::mt19937::result_type> dist_;
};
}  // namespace ufomap

#endif  // UFOMAP_OCTREE_RGB_H