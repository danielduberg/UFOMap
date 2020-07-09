#ifndef UFOMAP_OCTREE_BASE_H
#define UFOMAP_OCTREE_BASE_H

#include <ufomap/code.h>
#include <ufomap/iterator/leaf.h>
#include <ufomap/iterator/tree.h>
#include <ufomap/key.h>
#include <ufomap/node.h>
#include <ufomap/point_cloud.h>
#include <ufomap/types.h>

#include <algorithm>
#include <bitset>
#include <chrono>
#include <cstring>
#include <fstream>
#include <future>
#include <sstream>
#include <type_traits>

// Compression
#include <lz4.h>

namespace ufomap
{
template <typename LEAF_NODE,
					typename = std::enable_if_t<std::is_base_of_v<OccupancyNode, LEAF_NODE>>>
class OctreeBase
{
public:
	using Ray = std::vector<Point3>;
	template <typename BOUNDING_TYPE>
	using tree_iterator =
			TreeIterator<OctreeBase, InnerNode<LEAF_NODE>, LEAF_NODE, BOUNDING_TYPE>;
	template <typename BOUNDING_TYPE>
	using leaf_iterator =
			LeafIterator<OctreeBase, InnerNode<LEAF_NODE>, LEAF_NODE, BOUNDING_TYPE>;

public:
	virtual ~OctreeBase()
	{
		clear();
	}

	//
	// Tree type
	//

	virtual std::string getTreeType() const = 0;

	virtual std::string getTreeTypeOctomap() const = 0;

	//
	// Insertion
	//

	void insertRay(const Point3& origin, const Point3& end, double max_range = -1,
								 unsigned int depth = 0)
	{
		// Free space
		insertMissOnRay(origin, end, max_range, depth);
		// Occupied space
		integrateHit(end);
	}

	void insertMissOnRay(const Point3& origin, const Point3& end, double max_range = -1,
											 unsigned int depth = 0)

	{
		KeyRay ray;
		computeRay(origin, static_cast<const Point3&>(end), ray, max_range, depth);
		for (const Key& key : ray)
		{
			// Free space
			integrateMiss(key);
		}
	}

	void insertPointCloud(const Point3& sensor_origin, const PointCloud& cloud,
												double max_range = -1)
	{
		computeUpdate(sensor_origin, cloud, max_range);

		// Insert
		for (const auto& [code, value] : indices_)
		{
			updateNodeValue(code, value);
		}

		indices_.clear();
	}

	void insertPointCloudDiscrete(const Point3& sensor_origin, const PointCloud& cloud,
																double max_range = -1, unsigned int n = 0,
																unsigned int depth = 0)
	{
		KeyMap<std::vector<Key>> discrete_map;

		std::vector<Key> discrete;

		KeySet temp;
		Point3 origin;
		Point3 end;
		double distance;
		Point3 dir;
		Key changed_end;
		Point3 changed_point;
		Key point_key;

		if (insert_async_handler_[0].valid())
		{
			insert_async_handler_[0].wait();
		}

		for (const Point3& point : cloud)
		{
			point_key = coordToKey(static_cast<const Point3&>(point), 0);
			if (temp.insert(point_key).second)
			{
				changed_point = keyToCoord(point_key, 0);

				origin = sensor_origin;
				end = changed_point - origin;
				distance = end.norm();
				dir = end / distance;
				if (0 <= max_range && distance > max_range)
				{
					end = origin + (dir * max_range);
				}
				else
				{
					end = changed_point;
				}

				// Move origin and end to inside BBX
				if (!moveLineIntoBBX(origin, end))
				{
					// Line outside of BBX
					continue;
				}

				changed_end = coordToKey(end, 0);
				if (changed_point == end)
				{
					// if (0 == n && 0 != depth)  // TODO: Why 0 == depth? Should it not be 0 !=
					// 													 // depth
					// {
					// 	integrateHit(Code(changed_end));
					// }
					// else
					if (!indices_.try_emplace(changed_end, prob_hit_log_).second)
					{
						continue;
					}
				}

				discrete.push_back(changed_end);
			}
		}
		if (0 != depth)
		{
			std::vector<Key> previous;
			for (unsigned int d = (0 == n ? depth : 1); d <= depth; ++d)
			{
				previous.swap(discrete);
				discrete.clear();
				for (const Key& key : previous)
				{
					Key key_at_depth = Code(key).toDepth(d).toKey();
					std::vector<Key>& key_at_depth_children = discrete_map[key_at_depth];
					if (key_at_depth_children.empty())
					{
						discrete.push_back(key_at_depth);
					}
					key_at_depth_children.push_back(key);
				}
			}
		}

		computeUpdateDiscrete(sensor_origin, discrete, discrete_map, n);

		// Insert
		// for (const auto& [code, value] : indices_)
		// {
		// 	updateNodeValue(code, value);
		// }
		// indices_.clear();

		if (insert_async_handler_[1].valid())
		{
			insert_async_handler_[1].wait();
		}

		indices_buffer_.swap(indices_);

		insert_async_handler_[0] = std::async(std::launch::async, [this]() {
			indices_.clear();
			insert_async_handler_[1] = std::async(std::launch::async, [this]() {
				for (const auto& [code, value] : indices_buffer_)
				{
					updateNodeValue(code, value);
				}
			});
		});
	}

	void insertPointCloud(const Point3& sensor_origin, const PointCloud& cloud,
												const Pose6& frame_origin, double max_range = -1)
	{
		PointCloud cloud_transformed(cloud);
		cloud_transformed.transform(frame_origin);
		insertPointCloud(sensor_origin, cloud_transformed, max_range);
	}

	void insertPointCloudDiscrete(const Point3& sensor_origin, const PointCloud& cloud,
																const Pose6& frame_origin, double max_range = -1,
																unsigned int n = 0, unsigned int depth = 0)
	{
		PointCloud cloud_transformed(cloud);
		cloud_transformed.transform(frame_origin);
		insertPointCloudDiscrete(sensor_origin, cloud_transformed, max_range, n, depth);
	}

	//
	// Ray tracing
	//

	bool castRay(Point3 origin, Point3 direction, Point3& end, bool ignore_unknown = false,
							 double max_range = -1, unsigned int depth = 0) const
	{
		// TODO: Check so it is correct

		// Source: A Faster Voxel Traversal Algorithm for Ray Tracing
		Key current;
		Key ending;

		std::array<int, 3> step;
		Point3 t_delta;
		Point3 t_max;

		if (0 > max_range)
		{
			max_range = getMin().distance(getMax());
		}

		direction.normalize();
		Point3 the_end = origin + (direction * max_range);

		// Move origin and end to inside BBX
		if (!moveLineIntoBBX(origin, the_end))  // TODO: What should happen when origin is not
																						// in BBX?
		{
			// Line outside of BBX
			return false;
		}

		computeRayInit(origin, the_end, direction, current, ending, step, t_delta, t_max,
									 depth);

		// Increment
		while (current != ending && t_max.min() <= max_range && !isOccupied(current) &&
					 (ignore_unknown || !isUnknown(current)))
		{
			computeRayTakeStep(current, step, t_delta, t_max, depth);
		}

		// TODO: Set end correct?
		end = keyToCoord(current);

		return isOccupied(current);
	}

	void computeRay(const Point3& origin, const Point3& end, Ray& ray, double max_range = -1,
									unsigned int depth = 0) const
	{
		KeyRay key_ray;
		computeRay(origin, end, key_ray, max_range, depth);
		for (const Key& key : key_ray)
		{
			ray.push_back(keyToCoord(key));  // TODO: Ugly
		}
	}

	void computeRay(Point3 origin, Point3 end, KeyRay& ray, double max_range = -1,
									unsigned int depth = 0) const
	{
		// Source: A Faster Voxel Traversal Algorithm for Ray Tracing
		Key current;
		Key ending;

		std::array<int, 3> step;
		Point3 t_delta;
		Point3 t_max;

		Point3 direction = (end - origin).normalize();

		if (0 <= max_range && max_range < origin.distance(end))
		{
			end = origin + (direction * max_range);
		}

		// Move origin and end to inside BBX
		if (!moveLineIntoBBX(origin, end))
		{
			// Line outside of BBX
			return;
		}

		computeRayInit(origin, end, direction, current, ending, step, t_delta, t_max, depth);

		// Increment
		while (current != ending && t_max.min() <= max_range)
		{
			ray.push_back(current);
			computeRayTakeStep(current, step, t_delta, t_max, depth);
		}
	}

	// bool getRayIntersection(const Point3& origin, const Point3& direction,
	// 												const Point3& center, Point3& intersection, double delta) const
	// {
	// 	// TODO: Implement
	// 	return false;
	// }

	//
	// Pruning
	//
	void prune()
	{
		// TODO: Implement
	}

	//
	// Clear area
	//

	void clearAreaBBX(const Point3& bbx_min, const Point3& bbx_max, unsigned int depth = 0)
	{
		// TODO: Check if correct
		Key bbx_min_key = coordToKey(bbx_min, depth);
		Key bbx_max_key = coordToKey(bbx_max, depth);

		unsigned int inc = 1 << depth;

		for (unsigned int x = bbx_min_key[0]; x <= bbx_max_key[0]; x += inc)
		{
			for (unsigned int y = bbx_min_key[1]; y <= bbx_max_key[1]; y += inc)
			{
				for (unsigned int z = bbx_min_key[2]; z <= bbx_max_key[2]; z += inc)
				{
					setNodeValue(Key(x, y, z, depth), clamping_thres_min_log_);
				}
			}
		}
	}

	void clearAreaRadius(const Point3& coord, double radius, unsigned int depth = 0)
	{
		// TODO: Implement
	}

	//
	// Set node value
	//

	Node<LEAF_NODE> setNodeValue(const Node<LEAF_NODE>& node, double logit_value)
	{
		return setNodeValue(node.code, logit_value);  // TODO: Look at
	}

	Node<LEAF_NODE> setNodeValue(const Code& code, double logit_value)
	{
		logit_value =
				std::clamp(logit_value, clamping_thres_min_log_, clamping_thres_max_log_);

		Node<LEAF_NODE> node = getNode(code);
		if (logit_value != node.node->logit)
		{
			return updateNodeValueRecurs(code, logit_value, root_, depth_levels_, true).first;
		}
		return node;
	}

	Node<LEAF_NODE> setNodeValue(const Key& key, double logit_value)
	{
		return setNodeValue(Code(key), logit_value);
	}

	Node<LEAF_NODE> setNodeValue(const Point3& coord, double logit_value,
															 unsigned int depth = 0)
	{
		return setNodeValue(coordToKey(coord, depth), logit_value);
	}

	Node<LEAF_NODE> setNodeValue(double x, double y, double z, double logit_value,
															 unsigned int depth = 0)
	{
		return setNodeValue(coordToKey(x, y, z, depth), logit_value);
	}

	//
	// Update node value
	//

	Node<LEAF_NODE> updateNodeValue(const Node<LEAF_NODE>& node, double logit_update)
	{
		return updateNodeValue(node.code, logit_update);  // TODO: Look at
	}

	Node<LEAF_NODE> updateNodeValue(const Code& code, double logit_update)
	{
		Node<LEAF_NODE> node = getNode(code);
		if ((0 <= logit_update && node.node->logit >= clamping_thres_max_log_) ||
				(0 >= logit_update && node.node->logit <= clamping_thres_min_log_))
		{
			return node;
		}
		return updateNodeValueRecurs(code, logit_update, root_, depth_levels_).first;
	}

	Node<LEAF_NODE> updateNodeValue(const Key& key, double logit_update)
	{
		return updateNodeValue(Code(key), logit_update);
	}

	Node<LEAF_NODE> updateNodeValue(const Point3& coord, double logit_update,
																	unsigned int depth = 0)
	{
		return updateNodeValue(coordToKey(coord, depth), logit_update);
	}

	Node<LEAF_NODE> updateNodeValue(double x, double y, double z, double logit_update,
																	unsigned int depth = 0)
	{
		return updateNodeValue(coordToKey(x, y, z, depth), logit_update);
	}

	//
	// Integrate hit/miss
	//

	Node<LEAF_NODE> integrateHit(const Node<LEAF_NODE>& node)
	{
		return integrateHit(node.code);  // TODO: Look at
	}

	Node<LEAF_NODE> integrateHit(const Code& code)
	{
		return updateNodeValue(code, prob_hit_log_);
	}

	Node<LEAF_NODE> integrateHit(const Key& key)
	{
		return integrateHit(Code(key));
	}

	Node<LEAF_NODE> integrateHit(const Point3& coord, unsigned int depth = 0)
	{
		return integrateHit(coordToKey(coord, depth));
	}

	Node<LEAF_NODE> integrateHit(double x, double y, double z, unsigned int depth = 0)
	{
		return integrateHit(coordToKey(x, y, z, depth));
	}

	Node<LEAF_NODE> integrateMiss(const Node<LEAF_NODE>& node)
	{
		return integrateMiss(node.code);  // TODO: Look at
	}

	Node<LEAF_NODE> integrateMiss(const Code& code)
	{
		return updateNodeValue(code, prob_miss_log_);
	}

	Node<LEAF_NODE> integrateMiss(const Key& key)
	{
		return integrateMiss(Code(key));
	}

	Node<LEAF_NODE> integrateMiss(const Point3& coord, unsigned int depth = 0)
	{
		return integrateMiss(coordToKey(coord, depth));
	}

	Node<LEAF_NODE> integrateMiss(double x, double y, double z, unsigned int depth = 0)
	{
		return integrateMiss(coordToKey(x, y, z, depth));
	}

	//
	// Coordinate <-> key
	//

	inline unsigned int coordToKey(double coord, unsigned int depth = 0) const
	{
		int key_value = (int)floor(resolution_factor_ * coord);
		if (0 == depth)
		{
			return key_value + max_value_;
		}
		return ((key_value >> depth) << depth) + (1 << (depth - 1)) + max_value_;
	}

	inline Key coordToKey(const Point3& coord, unsigned int depth = 0) const
	{
		return Key(coordToKey(coord[0], depth), coordToKey(coord[1], depth),
							 coordToKey(coord[2], depth), depth);
	}

	inline Key coordToKey(double x, double y, double z, unsigned int depth = 0) const
	{
		return Key(coordToKey(x, depth), coordToKey(y, depth), coordToKey(z, depth), depth);
	}

	inline bool coordToKeyChecked(const Point3& coord, Key& key,
																unsigned int depth = 0) const
	{
		if (!inBBX(coord))
		{
			return false;
		}
		key = coordToKey(coord, depth);
		return true;
	}

	inline bool coordToKeyChecked(double x, double y, double z, Key& key,
																unsigned int depth = 0) const
	{
		if (!inBBX(x, y, z))
		{
			return false;
		}
		key = coordToKey(x, y, z, depth);
		return true;
	}

	inline double keyToCoord(KeyType key, unsigned int depth = 0) const
	{
		if (depth_levels_ == depth)
		{
			return 0.0;
		}

		// TODO: double or double?
		double divider = double(1 << depth);
		return (floor((double(key) - double(max_value_)) / divider) + 0.5) *
					 getNodeSize(depth);
	}

	inline Point3 keyToCoord(const Key& key) const
	{
		return Point3(keyToCoord(key[0], key.getDepth()), keyToCoord(key[1], key.getDepth()),
									keyToCoord(key[2], key.getDepth()));
	}

	inline Point3 keyToCoord(const Key& key, unsigned int depth) const
	{
		return Point3(keyToCoord(key[0], depth), keyToCoord(key[1], depth),
									keyToCoord(key[2], depth));
	}

	inline bool keyToCoordChecked(const Key& key, Point3& coord, unsigned int depth) const
	{
		if (key.getDepth() > depth)
		{
			return false;
		}
		coord = keyToCoord(key, depth);
		return true;
	}

	//
	// Checking state of node
	//

	bool isOccupied(const Node<LEAF_NODE>& node) const
	{
		return isOccupiedLog(node.node->logit);
	}

	bool isOccupied(const Code& code) const
	{
		return isOccupied(getNode(code));
	}

	bool isOccupied(const Key& key) const
	{
		return isOccupied(Code(key));
	}

	bool isOccupied(const Point3& coord, unsigned int depth = 0) const
	{
		return isOccupied(coordToKey(coord, depth));
	}

	bool isOccupied(double x, double y, double z, unsigned int depth = 0) const
	{
		return isOccupied(coordToKey(x, y, z, depth));
	}

	bool isOccupied(double probability) const
	{
		return isOccupied(logit(probability));
	}

	bool isOccupiedLog(double logit) const
	{
		return occupancy_thres_log_ < logit;
	}

	bool isFree(const Node<LEAF_NODE>& node) const
	{
		return isFreeLog(node.node->logit);
	}

	bool isFree(const Code& code) const
	{
		return isFree(getNode(code));
	}

	bool isFree(const Key& key) const
	{
		return isFree(Code(key));
	}

	bool isFree(const Point3& coord, unsigned int depth = 0) const
	{
		return isFree(coordToKey(coord, depth));
	}

	bool isFree(double x, double y, double z, unsigned int depth = 0) const
	{
		return isFree(coordToKey(x, y, z, depth));
	}

	bool isFree(double probability) const
	{
		return isFree(logit(probability));
	}

	bool isFreeLog(double logit) const
	{
		return free_thres_log_ > logit;
	}

	bool isUnknown(const Node<LEAF_NODE>& node) const
	{
		return isUnknownLog(node.node->logit);
	}

	bool isUnknown(const Code& code) const
	{
		return isUnknown(getNode(code));
	}

	bool isUnknown(const Key& key) const
	{
		return isUnknown(Code(key));
	}

	bool isUnknown(const Point3& coord, unsigned int depth = 0) const
	{
		return isUnknown(coordToKey(coord, depth));
	}

	bool isUnknown(double x, double y, double z, unsigned int depth = 0) const
	{
		return isUnknown(coordToKey(x, y, z, depth));
	}

	bool isUnknown(double probability) const
	{
		return isUnknown(logit(probability));
	}

	bool isUnknownLog(double logit) const
	{
		return free_thres_log_ <= logit && occupancy_thres_log_ >= logit;
	}

	bool containsOccupied(const Node<LEAF_NODE>& node) const
	{
		return isOccupied(node);
	}

	bool containsOccupied(const Code& code) const
	{
		return isOccupied(code);
	}

	bool containsOccupied(const Key& key) const
	{
		return isOccupied(key);
	}

	bool containsOccupied(const Point3& coord, unsigned int depth = 0) const
	{
		return isOccupied(coord, depth);
	}

	bool containsOccupied(double x, double y, double z, unsigned int depth = 0) const
	{
		return isOccupied(x, y, z, depth);
	}

	bool containsFree(const Node<LEAF_NODE>& node) const
	{
		if (0 == node.getDepth())
		{
			return isFree(node);
		}
		else
		{
			return static_cast<const InnerNode<LEAF_NODE>*>(node.node)->contains_free;
		}
	}

	bool containsFree(const Code& code) const
	{
		return containsFree(getNode(code));
	}

	bool containsFree(const Key& key) const
	{
		return containsFree(Code(key));
	}

	bool containsFree(const Point3& coord, unsigned int depth = 0) const
	{
		return containsFree(coordToKey(coord, depth));
	}

	bool containsFree(double x, double y, double z, unsigned int depth = 0) const
	{
		return containsFree(coordToKey(x, y, z, depth));
	}

	bool containsUnknown(const Node<LEAF_NODE>& node) const
	{
		if (0 == node.getDepth())
		{
			return isUnknown(node);
		}
		else
		{
			return static_cast<const InnerNode<LEAF_NODE>*>(node.node)->contains_unknown;
		}
	}

	bool containsUnknown(const Code& code) const
	{
		return containsUnknown(getNode(code));
	}

	bool containsUnknown(const Key& key) const
	{
		return containsUnknown(Code(key));
	}

	bool containsUnknown(const Point3& coord, unsigned int depth = 0) const
	{
		return containsUnknown(coordToKey(coord, depth));
	}

	bool containsUnknown(double x, double y, double z, unsigned int depth = 0) const
	{
		return containsUnknown(coordToKey(x, y, z, depth));
	}

	//
	// Iterators
	//

	// TODO: Update to use AABB version?
	tree_iterator<ufomap_geometry::AABB> begin_tree(bool occupied_space = true,
																									bool free_space = true,
																									bool unknown_space = false,
																									bool contains = false,
																									unsigned int min_depth = 0) const
	{
		return tree_iterator<ufomap_geometry::AABB>(
				this, ufomap_geometry::AABB(getMin(), getMax()), occupied_space, free_space,
				unknown_space, contains, min_depth);
	}

	const tree_iterator<ufomap_geometry::AABB> end_tree() const
	{
		return tree_iterator<ufomap_geometry::AABB>();
	}

	template <typename BOUNDING_TYPE>
	tree_iterator<BOUNDING_TYPE>
	begin_tree_bounding(const BOUNDING_TYPE& bounding_type, bool occupied_space = true,
											bool free_space = true, bool unknown_space = false,
											bool contains = false, unsigned int min_depth = 0) const
	{
		return tree_iterator<BOUNDING_TYPE>(this, bounding_type, occupied_space, free_space,
																				unknown_space, contains, min_depth);
	}

	// TODO: Update to use AABB version?
	leaf_iterator<ufomap_geometry::AABB> begin_leafs(bool occupied_space = true,
																									 bool free_space = true,
																									 bool unknown_space = false,
																									 bool contains = false,
																									 unsigned int min_depth = 0) const
	{
		return leaf_iterator<ufomap_geometry::AABB>(
				this, ufomap_geometry::AABB(getMin(), getMax()), occupied_space, free_space,
				unknown_space, contains, min_depth);
	}

	template <typename BOUNDING_TYPE = ufomap_geometry::AABB>
	const leaf_iterator<BOUNDING_TYPE> end_leafs() const
	{
		return leaf_iterator<BOUNDING_TYPE>();
	}

	template <typename BOUNDING_TYPE>
	leaf_iterator<BOUNDING_TYPE>
	begin_leafs_bounding(const BOUNDING_TYPE& bounding_type, bool occupied_space = true,
											 bool free_space = true, bool unknown_space = false,
											 bool contains = false, unsigned int min_depth = 0) const
	{
		return leaf_iterator<BOUNDING_TYPE>(this, bounding_type, occupied_space, free_space,
																				unknown_space, contains, min_depth);
	}

	//
	// BBX
	//

	Point3 getBBXBounds() const
	{
		return (bbx_max_ - bbx_min_) / 2.0;
	}

	Point3 getBBXCenter() const
	{
		return bbx_min_ + ((bbx_max_ - bbx_min_) / 2.0);
	}

	Point3 getBBXMin() const
	{
		return bbx_min_;
	}

	Point3 getBBXMax() const
	{
		return bbx_max_;
	}

	bool inBBX(const Key& key) const
	{
		Key min = isBBXLimitEnabled() ? bbx_min_key_ : coordToKey(getMin());
		Key max = isBBXLimitEnabled() ? bbx_max_key_ : coordToKey(getMax());
		return min[0] <= key[0] && max[0] >= key[0] && min[1] <= key[1] && max[1] >= key[1] &&
					 min[2] <= key[2] && max[2] >= key[2];
	}

	bool inBBX(const Point3& coord) const
	{
		return inBBX(coord.x(), coord.y(), coord.z());
	}

	bool inBBX(double x, double y, double z) const
	{
		Point3 min = isBBXLimitEnabled() ? bbx_min_ : getMin();
		Point3 max = isBBXLimitEnabled() ? bbx_max_ : getMax();
		return min.x() <= x && max.x() >= x && min.y() <= y && max.y() >= y && min.z() <= z &&
					 max.z() >= z;
	}

	void setBBXMin(const Point3& min)
	{
		bbx_min_ = min;
	}

	void setBBXMax(const Point3& max)
	{
		bbx_max_ = max;
	}

	void enableBBXLimit(bool enable)
	{
		bbx_limit_enabled_ = enable;
	}

	bool isBBXLimitEnabled() const
	{
		return bbx_limit_enabled_;
	}

	bool moveLineIntoBBX(const Point3& bbx_min, const Point3& bbx_max, Point3& origin,
											 Point3& end) const
	{
		if ((origin[0] < bbx_min[0] && end[0] < bbx_min[0]) ||
				(origin[0] > bbx_max[0] && end[0] > bbx_max[0]) ||
				(origin[1] < bbx_min[1] && end[1] < bbx_min[1]) ||
				(origin[1] > bbx_max[1] && end[1] > bbx_max[1]) ||
				(origin[2] < bbx_min[2] && end[2] < bbx_min[2]) ||
				(origin[2] > bbx_max[2] && end[2] > bbx_max[2]))
		{
			return false;
		}

		int hits = 0;
		std::array<Point3, 2> hit;
		for (int i = 0; i < 3 && hits < 2; ++i)
		{
			if (getIntersection(origin[i] - bbx_min[i], end[i] - bbx_min[i], origin, end,
													&hit[hits]) &&
					inBBX(hit[hits], i, bbx_min, bbx_max))
			{
				++hits;
			}
		}
		for (int i = 0; i < 3 && hits < 2; ++i)
		{
			if (getIntersection(origin[i] - bbx_max[i], end[i] - bbx_max[i], origin, end,
													&hit[hits]) &&
					inBBX(hit[hits], i, bbx_min, bbx_max))
			{
				++hits;
			}
		}

		switch (hits)
		{
			case 1:
				if (inBBX(origin))
				{
					end = hit[0];
				}
				else
				{
					origin = hit[0];
				}
				break;
			case 2:
				if (((origin - hit[0]).squaredNorm() + (end - hit[1]).squaredNorm()) <=
						((origin - hit[1]).squaredNorm() + (end - hit[0]).squaredNorm()))
				{
					origin = hit[0];
					end = hit[1];
				}
				else
				{
					origin = hit[1];
					end = hit[0];
				}
		}

		return true;
	}

	bool moveLineIntoBBX(Point3& origin, Point3& end) const
	{
		Point3 bbx_min = isBBXLimitEnabled() ? bbx_min_ : getMin();
		Point3 bbx_max = isBBXLimitEnabled() ? bbx_max_ : getMax();
		return moveLineIntoBBX(bbx_min, bbx_max, origin, end);
	}

	//
	// Sensor model functions
	//

	double logit(const Node<LEAF_NODE>& node) const
	{
		return node.node->logit;
	}

	double logit(double probability) const
	{
		return std::log(probability / (1.0 - probability));
	}

	double probability(const Node<LEAF_NODE>& node) const
	{
		return probability(node.node->logit);
	}

	double probability(double logit) const
	{
		return 1.0 - (1.0 / (1.0 + std::exp(logit)));
	}

	double getOccupancyThres() const
	{
		return probability(occupancy_thres_log_);
	}

	double getOccupancyThresLog() const
	{
		return occupancy_thres_log_;
	}

	double getFreeThres() const
	{
		return probability(free_thres_log_);
	}

	double getFreeThresLog() const
	{
		return free_thres_log_;
	}

	double getProbHit() const
	{
		return probability(prob_hit_log_);
	}

	double getProbHitLog() const
	{
		return prob_hit_log_;
	}

	double getProbMiss() const
	{
		return probability(prob_miss_log_);
	}

	double getProbMissLog() const
	{
		return prob_miss_log_;
	}

	double getClampingThresMin() const
	{
		return probability(clamping_thres_min_log_);
	}

	double getClampingThresMinLog() const
	{
		return clamping_thres_min_log_;
	}

	double getClampingThresMax() const
	{
		return probability(clamping_thres_max_log_);
	}

	double getClampingThresMaxLog() const
	{
		return clamping_thres_max_log_;
	}

	// TODO: Should add a warning that these are very computational expensive to call since
	// the whole tree has to be updated
	void setOccupancyThres(double probability)
	{
		setOccupancyThresLog(logit(probability));
	}

	void setOccupancyThresLog(double logit)
	{
		occupancy_thres_log_ = logit;
		// TODO: Update tree
	}

	// TODO: Should add a warning that these are very computational expensive to call since
	// the whole tree has to be updated
	void setFreeThres(double probability)
	{
		setFreeThresLog(logit(probability));
	}

	void setFreeThresLog(double logit)
	{
		free_thres_log_ = logit;
		// TODO: Update tree
	}

	void setProbHit(double probability)
	{
		setProbHitLog(logit(probability));
	}

	void setProbHitLog(double logit)
	{
		prob_hit_log_ = logit;
	}

	void setProbMiss(double probability)
	{
		setProbMissLog(logit(prob_hit_log_));
	}

	void setProbMissLog(double logit)
	{
		prob_miss_log_ = logit;
	}

	void setClampingThresMin(double probability)
	{
		setClampingThresMinLog(logit(probability));
	}

	void setClampingThresMinLog(double logit)
	{
		clamping_thres_min_log_ = logit;
	}

	void setClampingThresMax(double probability)
	{
		setClampingThresMaxLog(logit(probability));
	}

	void setClampingThresMaxLog(double logit)
	{
		clamping_thres_max_log_ = logit;
	}

	//
	// Memory functions
	//

	/**
	 * @return size_t number of nodes in the tree
	 */
	size_t size() const
	{
		return num_inner_nodes_ + num_inner_leaf_nodes_ + num_leaf_nodes_;
	}

	/**
	 * @return size_t memory usage of the octree
	 */
	size_t memoryUsage() const
	{
		return (num_inner_nodes_ * memoryUsageInnerNode()) +
					 (num_inner_leaf_nodes_ * memoryUsageInnerLeafNode()) +
					 (num_leaf_nodes_ * memoryUsageLeafNode());
	}

	/**
	 * @return size_t memory usage of a single inner node
	 */
	size_t memoryUsageInnerNode() const
	{
		return sizeof(InnerNode<LEAF_NODE>);
	}

	/**
	 * @return size_t memory usage of a single inner leaf node
	 */
	size_t memoryUsageInnerLeafNode() const
	{
		return sizeof(InnerNode<LEAF_NODE>);
	}

	/**
	 * @return size_t memory usage of a single leaf node
	 */
	size_t memoryUsageLeafNode() const
	{
		return sizeof(LEAF_NODE);
	}

	// /**
	//  * @return unsigned long long Memory usage if the tree contained the maximum number
	//  of
	//  * nodes
	//  */
	// unsigned long long memoryFullGrid() const;

	/**
	 * @return size_t number of inner nodes in the tree
	 */
	size_t getNumInnerNodes() const
	{
		return num_inner_nodes_;
	}

	/**
	 * @return size_t number of inner leaf nodes in the tree
	 */
	size_t getNumInnerLeafNodes() const
	{
		return num_inner_leaf_nodes_;
	}

	/**
	 * @return size_t number of leaf nodes in the tree
	 */
	size_t getNumLeafNodes() const
	{
		return num_leaf_nodes_;
	}

	//
	// Metrics
	//

	double volume() const
	{
		Point3 size = getMetricSize();
		return size[0] * size[1] * size[2];
	}

	/**
	 * @brief Size of all known space (occupied or free) in meters for x, y, and z
	 * dimension.
	 *
	 * @return Point3 size of all known space in meter in x, y, and z dimension
	 */
	Point3 getMetricSize(unsigned int depth = 0) const
	{
		return getMetricMax(depth) - getMetricMin(depth);
	}

	/**
	 * @return Minimum value of the bounding box of all known space
	 */
	Point3 getMetricMin(unsigned int depth = 0) const
	{
		// TODO: Precompute this?
		Point3 min_coord = getMax();

		for (auto it = begin_leafs(true, true, false, false, depth), end = end_leafs();
				 it != end; ++it)
		{
			Point3 center = it.getCenter();
			double half_size = it.getHalfSize();
			min_coord.x() = std::min(min_coord.x(), center.x() - half_size);
			min_coord.y() = std::min(min_coord.y(), center.y() - half_size);
			min_coord.z() = std::min(min_coord.z(), center.z() - half_size);
		}
		return min_coord;
	}

	/**
	 * @return Maximum value of the bounding box of all known space
	 */
	Point3 getMetricMax(unsigned int depth = 0) const
	{
		// TODO: Precompute this?
		Point3 max_coord = getMin();

		for (auto it = begin_leafs(true, true, false, false, depth), end = end_leafs();
				 it != end; ++it)
		{
			Point3 center = it.getCenter();
			double half_size = it.getHalfSize();
			max_coord.x() = std::max(max_coord.x(), center.x() + half_size);
			max_coord.y() = std::max(max_coord.y(), center.y() + half_size);
			max_coord.z() = std::max(max_coord.z(), center.z() + half_size);
		}
		return max_coord;
	}

	/**
	 * @return The minimum point the octree can store
	 */
	Point3 getMin() const
	{
		double half_size = -getNodeHalfSize(depth_levels_);
		return Point3(half_size, half_size, half_size);
	}

	/**
	 * @return The maximum point the octree can store
	 */
	Point3 getMax() const
	{
		double half_size = getNodeHalfSize(depth_levels_);
		return Point3(half_size, half_size, half_size);
	}

	//
	// Change detection
	//
	void enableChangeDetection(bool enable)
	{
		change_detection_enabled_ = enable;
	}

	bool isChangeDetectionEnabled() const
	{
		return change_detection_enabled_;
	}

	void resetChangeDetection()
	{
		changed_codes_.clear();
	}

	size_t numChangesDeteced() const
	{
		return changed_codes_.size();
	}

	const CodeSet& getChangedCodes() const
	{
		return changed_codes_;
	}

	CodeSet getChangedCodes()
	{
		return changed_codes_;
	}

	CodeSet::const_iterator changedCodesBegin() const
	{
		return changed_codes_.cbegin();
	}

	CodeSet::const_iterator changedCodesEnd() const
	{
		return changed_codes_.cend();
	}

	//
	// Clear
	//

	void clear()
	{
		clear(resolution_, depth_levels_);
	}

	void clear(double resolution, unsigned int depth_levels)
	{
		if (21 < depth_levels)
		{
			throw std::invalid_argument("depth_levels can be maximum 21");
		}

		clear(root_, depth_levels_);
		root_ = InnerNode<LEAF_NODE>();

		depth_levels_ = depth_levels;
		max_value_ = std::pow(2, depth_levels - 1);

		if (resolution != resolution_)
		{
			resolution_ = resolution;
			resolution_factor_ = 1.0 / resolution;

			nodes_sizes_.resize(depth_levels_ + 1);
			nodes_sizes_[0] = resolution_;
			nodes_half_sizes_.resize(depth_levels_ + 1);
			nodes_half_sizes_[0] = resolution_ / 2.0;
			for (size_t i = 1; i <= depth_levels_; ++i)
			{
				nodes_sizes_[i] = nodes_sizes_[i - 1] * 2.0;
				nodes_half_sizes_[i] = nodes_sizes_[i - 1];
			}
		}
	}

	//
	// Node functions
	//

	Node<LEAF_NODE> getRoot() const
	{
		return Node<LEAF_NODE>(&root_, Code(0, depth_levels_));  // TODO: Check if correct
	}

	Node<LEAF_NODE> getNode(const Code& code, bool return_nullptr = false) const
	{
		const LEAF_NODE* current_node = &root_;

		for (unsigned int depth = depth_levels_; depth > code.getDepth(); --depth)
		{
			const InnerNode<LEAF_NODE>* inner_node =
					static_cast<const InnerNode<LEAF_NODE>*>(current_node);

			if (!hasChildren(*inner_node))
			{
				if (return_nullptr)
				{
					return Node<LEAF_NODE>(nullptr, Code());  // TODO: Correct?;
				}
				else
				{
					return Node<LEAF_NODE>(current_node,
																 code.toDepth(depth + 1));  // TODO: Correct?
				}
			}

			// Get child index
			unsigned int child_idx = code.getChildIdx(depth - 1);

			current_node = (1 == depth) ? &(*static_cast<std::array<LEAF_NODE, 8>*>(
																				inner_node->children))[child_idx] :
																		&(*static_cast<std::array<InnerNode<LEAF_NODE>, 8>*>(
																				inner_node->children))[child_idx];
		}

		return Node<LEAF_NODE>(current_node, code);
	}

	Node<LEAF_NODE> getNode(const Key& key) const
	{
		return getNode(Code(key));
	}

	Node<LEAF_NODE> getNode(const Point3& coord, unsigned int depth = 0) const
	{
		return getNode(coordToKey(coord, depth));
	}

	Node<LEAF_NODE> getNode(double x, double y, double z, unsigned int depth = 0) const
	{
		return getNode(coordToKey(x, y, z, depth));
	}

	double getNodeSize(unsigned int depth) const
	{
		return nodes_sizes_[depth];
	}

	double getNodeHalfSize(unsigned int depth) const
	{
		return nodes_half_sizes_[depth];
	}

	//
	// Checking for children
	//

	bool isLeaf(const Node<LEAF_NODE>& node) const
	{
		if (0 == node.getDepth())
		{
			return true;
		}
		return isLeaf(*static_cast<const InnerNode<LEAF_NODE>*>(node.node));
	}

	bool hasChildren(const Node<LEAF_NODE>& node) const
	{
		if (0 == node.getDepth())
		{
			return false;
		}
		return hasChildren(*static_cast<const InnerNode<LEAF_NODE>*>(node.node));
	}

	//
	// Random functions
	//

	unsigned int getTreeDepthLevels() const
	{
		return depth_levels_;
	}

	double getResolution() const
	{
		return resolution_;
	}

	void setAutomaticPruning(bool enable_automatic_pruning)
	{
		automatic_pruning_enabled_ = enable_automatic_pruning;
	}

	bool isAutomaticPruningEnabled() const
	{
		return automatic_pruning_enabled_;
	}

	//
	// Read/write
	//

	bool read(const std::string& filename)
	{
		std::ifstream file(filename.c_str(), std::ios_base::in | std::ios_base::binary);
		if (!file.is_open())
		{
			// Error
			return false;
		}
		// TODO: check is_good of finished stream, warn?
		return read(file);
	}

	bool read(std::istream& s)
	{
		auto [type, binary] = readFirstLineHeader(s);

		if ("" == type)
		{
			return false;
		}

		if (binary && !binarySupport())
		{
			return false;
		}

		bool is_ufomap = "ufomap" == type;

		std::string id;
		size_t size;
		double res;
		unsigned int depth_levels;
		double occupancy_thres;
		double free_thres;
		bool compressed;
		int data_size;
		int compressed_data_size;
		if (!readHeader(s, id, size, res, depth_levels, occupancy_thres, free_thres,
										compressed, data_size, compressed_data_size, is_ufomap))
		{
			return false;
		}

		if (compressed)
		{
			std::stringstream uncompressed_s(std::ios_base::in | std::ios_base::out |
																			 std::ios_base::binary);
			if (decompressData(s, uncompressed_s, data_size, compressed_data_size))
			{
				return readData(uncompressed_s, res, depth_levels, occupancy_thres, free_thres,
												binary, !is_ufomap);
			}
			return false;
		}
		else
		{
			return readData(s, res, depth_levels, occupancy_thres, free_thres, binary,
											!is_ufomap);
		}
		// TODO: Check size?
	}

	bool readData(std::istream& s, double resolution, unsigned int depth_levels,
								double occupancy_thres, double free_thres, bool binary = false,
								bool from_octomap = false)
	{
		if (binary && !binarySupport())
		{
			return false;
		}

		if (!s.good())
		{
			// Warning
		}

		clear(resolution, depth_levels);

		root_ = InnerNode<LEAF_NODE>();

		if (binary)
		{
			return readBinaryNodesRecurs(s, root_, depth_levels_, logit(occupancy_thres),
																	 logit(free_thres), from_octomap);
		}
		else
		{
			return readNodesRecurs(s, root_, depth_levels_, logit(occupancy_thres),
														 logit(free_thres), from_octomap);
		}
	}

	bool decompressData(std::istream& s_in, std::iostream& s_out, int data_size,
											int compressed_data_size) const
	{
		char* compressed_data = new char[compressed_data_size];
		s_in.read(compressed_data, compressed_data_size);
		char* regen_buffer = new char[data_size];
		const int decompressed_size = LZ4_decompress_safe(compressed_data, regen_buffer,
																											compressed_data_size, data_size);
		if (0 > decompressed_size)
		{
			return false;
		}
		s_out.write(regen_buffer, decompressed_size);
		delete[] compressed_data;
		delete[] regen_buffer;
		return true;
	}

	bool readDataCompressed(std::istream& s, double resolution, unsigned int depth_levels,
													double occupancy_thres, double free_thres, int data_size,
													int compressed_data_size, bool binary = false,
													bool from_octomap = false)
	{
		std::stringstream uncompressed_s(std::ios_base::in | std::ios_base::out |
																		 std::ios_base::binary);
		if (decompressData(s, uncompressed_s, data_size, compressed_data_size))
		{
			return readData(uncompressed_s, resolution, depth_levels, occupancy_thres,
											free_thres, binary, from_octomap);
		}
		return false;
	}

	bool write(const std::string& filename, bool compress = false, bool binary = false,
						 bool to_octomap = false) const
	{
		if (binary && !binarySupport())
		{
			return false;
		}

		std::ofstream file(filename.c_str(), std::ios_base::out | std::ios_base::binary);

		if (!file.is_open())
		{
			// ERROR
			return false;
		}
		// TODO: check is_good of finished stream, return
		write(file, compress, binary, to_octomap);
		file.close();
		return true;
	}

	bool write(std::ostream& s, bool compress = false, bool binary = false,
						 bool to_octomap = false) const
	{
		if (binary && !binarySupport())
		{
			return false;
		}

		if (to_octomap && (16 != depth_levels_ || "" == getTreeTypeOctomap()))
		{
			return false;
		}

		if (to_octomap)
		{
			s << (binary ? BINARY_FILE_HEADER_OCTOMAP : FILE_HEADER_OCTOMAP);
		}
		else
		{
			s << (binary ? BINARY_FILE_HEADER : FILE_HEADER);
		}

		s << "\n# (feel free to add / change comments, but leave the first line as it "
				 "is!)\n#\n";
		if (to_octomap)
		{
			s << "id " << getTreeTypeOctomap() << std::endl;
		}
		else
		{
			s << "id " << getTreeType() << std::endl;
		}
		s << "size " << size() << std::endl;
		s << "res " << getResolution() << std::endl;
		s << "depth_levels " << getTreeDepthLevels() << std::endl;
		s << "occupancy_thres " << getOccupancyThres() << std::endl;
		s << "free_thres " << getFreeThres() << std::endl;
		s << "compressed " << compress << std::endl;

		std::stringstream data(std::ios_base::in | std::ios_base::out |
													 std::ios_base::binary);
		std::stringstream compressed_data(std::ios_base::in | std::ios_base::out |
																			std::ios_base::binary);
		writeData(data, binary, to_octomap);
		if (compress)
		{
			compressData(data, compressed_data);
		}
		s << "data_size " << getDataSize(data) << std::endl;
		s << "compressed_data_size " << getDataSize(compressed_data) << std::endl;
		s << "data" << std::endl;
		s << (compress ? compressed_data.rdbuf() : data.rdbuf());

		if (s.good())
		{
			return true;
		}
		return false;
	}

	bool writeData(std::ostream& s, bool binary = false, bool to_octomap = false) const
	{
		if (binary && !binarySupport())
		{
			return false;
		}

		if (binary)
		{
			return writeBinaryNodesRecurs(s, root_, depth_levels_, to_octomap);
		}
		else
		{
			return writeNodesRecurs(s, root_, depth_levels_, to_octomap);
		}
	}

	std::pair<int, int> compressData(std::istream& s_in, std::ostream& s_out) const
	{
		const int data_size = getDataSize(s_in);
		char* data = new char[data_size];
		s_in.read(data, data_size);
		const int max_dst_size = LZ4_compressBound(data_size);
		char* compressed_data = new char[max_dst_size];
		const int compressed_data_size =
				LZ4_compress_default(data, compressed_data, data_size, max_dst_size);
		s_out.write(compressed_data, compressed_data_size);
		delete[] compressed_data;
		delete[] data;
		return std::make_pair(data_size, compressed_data_size);
	}

	bool writeDataCompress(std::ostream& s, int& data_size, int& compressed_data_size,
												 bool binary = false, bool to_octomap = false) const
	{
		std::stringstream data(std::ios_base::in | std::ios_base::out |
													 std::ios_base::binary);
		if (!writeData(data, binary, to_octomap))
		{
			return false;
		}
		std::tie(data_size, compressed_data_size) = compressData(data, s);
		return true;
	}

protected:
	OctreeBase(double resolution, unsigned int depth_levels, bool automatic_pruning,
						 double occupancy_thres, double free_thres, double prob_hit, double prob_miss,
						 double clamping_thres_min, double clamping_thres_max)
		: resolution_(resolution)
		, resolution_factor_(1.0 / resolution)
		, depth_levels_(depth_levels)
		, max_value_(std::pow(2, depth_levels - 1))
		, occupancy_thres_log_(logit(occupancy_thres))
		, free_thres_log_(logit(free_thres))
		, prob_hit_log_(logit(prob_hit))
		, prob_miss_log_(logit(prob_miss))
		, clamping_thres_min_log_(logit(clamping_thres_min))
		, clamping_thres_max_log_(logit(clamping_thres_max))
		, automatic_pruning_enabled_(automatic_pruning)
	{
		if (21 < depth_levels)
		{
			throw std::invalid_argument("depth_levels can be maximum 21");
		}

		nodes_sizes_.reserve(depth_levels_ + 1);
		nodes_sizes_.push_back(resolution_);
		nodes_half_sizes_.reserve(depth_levels_ + 1);
		nodes_half_sizes_.push_back(resolution_ / 2.0);
		for (size_t i = 1; i <= depth_levels_; ++i)
		{
			nodes_sizes_.push_back(nodes_sizes_[i - 1] * 2.0);
			nodes_half_sizes_.push_back(nodes_sizes_[i - 1]);
		}

		indices_.max_load_factor(0.8);
		indices_.reserve(100003);
		indices_buffer_.max_load_factor(0.8);
		indices_buffer_.reserve(100003);
		// discretize_.max_load_factor(0.8);
		// discretize_.reserve(10007);
	}

	//
	// Update node value
	//

	std::pair<Node<LEAF_NODE>, bool> updateNodeValueRecurs(const Code& code,
																												 double logit_value,
																												 LEAF_NODE& node,
																												 unsigned int current_depth,
																												 bool set_value = false)
	{
		if (current_depth > code.getDepth())
		{
			InnerNode<LEAF_NODE>& inner_node = static_cast<InnerNode<LEAF_NODE>&>(node);

			// Create children if they do not exist
			expand(inner_node, current_depth);

			unsigned int child_depth = current_depth - 1;

			// Get child index
			unsigned int child_idx = code.getChildIdx(child_depth);

			// Get child
			LEAF_NODE* child_node = (0 == child_depth) ?
																	&(getLeafChildren(inner_node))[child_idx] :
																	&(getInnerChildren(inner_node))[child_idx];

			auto [child, changed] =
					updateNodeValueRecurs(code, logit_value, *child_node, child_depth, set_value);

			// Recurs
			if (changed)
			{
				// Update this node
				changed = updateNode(inner_node, current_depth);
				if (changed && change_detection_enabled_)
				{
					changed_codes_.insert(code.toDepth(current_depth));
				}
			}

			if (hasChildren(inner_node))
			{
				return std::make_pair(child, changed);
			}
			else
			{
				return std::make_pair(Node<LEAF_NODE>(&node, current_depth), changed);
			}
		}
		else
		{
			// TODO: check

			if (set_value)
			{
				node.logit =
						std::clamp(logit_value, clamping_thres_min_log_, clamping_thres_max_log_);

				if (0 < current_depth)
				{
					InnerNode<LEAF_NODE>& inner_node = static_cast<InnerNode<LEAF_NODE>&>(node);
					prune(inner_node, current_depth);
					updateNode(inner_node, current_depth);
				}
			}
			else
			{
				if (0 < current_depth)
				{
					InnerNode<LEAF_NODE>& inner_node = static_cast<InnerNode<LEAF_NODE>&>(node);

					// // Update value
					// inner_node.logit = std::clamp(inner_node.logit + logit_value,
					// 															clamping_thres_min_log_,
					// clamping_thres_max_log_);

					// if (!isOccupiedLog(inner_node.logit))
					// {
					// 	prune(inner_node, current_depth);
					// }
					// else if (hasChildren(inner_node))
					if (hasChildren(inner_node))
					{
						unsigned int child_depth = current_depth - 1;
						for (unsigned int child_idx = 0; child_idx < 8; ++child_idx)
						{
							LEAF_NODE& child_node = (0 == child_depth) ?
																					getLeafChildren(inner_node)[child_idx] :
																					getInnerChildren(inner_node)[child_idx];
							updateNodeValueRecurs(code.getChild(child_idx), logit_value, child_node,
																		child_depth, set_value);
						}
					}
					else
					{
						// Update value
						inner_node.logit =
								std::clamp(inner_node.logit + logit_value, clamping_thres_min_log_,
													 clamping_thres_max_log_);
					}

					// Update this node
					updateNode(inner_node, current_depth);
				}
				else
				{
					// Update value
					node.logit = std::clamp(node.logit + logit_value, clamping_thres_min_log_,
																	clamping_thres_max_log_);
				}
			}

			if (change_detection_enabled_)
			{
				changed_codes_.insert(code);
			}

			return std::make_pair(Node<LEAF_NODE>(&node, code), true);
		}
	}

	//
	// Update node
	//

	virtual bool updateNode(InnerNode<LEAF_NODE>& node, unsigned int depth)
	{
		if (!hasChildren(node))
		{
			bool new_contains_free = isFreeLog(node.logit);
			bool new_contains_unknown = isUnknownLog(node.logit);
			bool updated = (node.contains_free != new_contains_free) ||
										 (node.contains_unknown != new_contains_unknown);
			node.contains_free = new_contains_free;
			node.contains_unknown = new_contains_unknown;
			return updated;
		}
		else if (1 == depth)
		{
			return updateNode(node, getLeafChildren(node), depth);
		}
		else
		{
			return updateNode(node, getInnerChildren(node), depth);
		}
	}

	virtual bool updateNode(InnerNode<LEAF_NODE>& node,
													const std::array<LEAF_NODE, 8>& children, unsigned int depth)
	{
		if (isNodeCollapsible(children))
		{
			// Note: Can not assume that these are the same in this function
			node.logit = children[0].logit;
			prune(node, depth);
			return true;
		}

		double new_logit = getMaxChildLogit(children);
		bool new_contains_free = false;
		bool new_contains_unknown = false;
		for (const LEAF_NODE& child : children)
		{
			if (isFree(child))
			{
				new_contains_free = true;
			}
			else if (isUnknown(child))
			{
				new_contains_unknown = true;
			}
		}

		if (node.logit != new_logit || node.contains_free != new_contains_free ||
				node.contains_unknown != new_contains_unknown)
		{
			node.logit = new_logit;
			node.contains_free = new_contains_free;
			node.contains_unknown = new_contains_unknown;
			return true;
		}
		return false;
	}

	virtual bool updateNode(InnerNode<LEAF_NODE>& node,
													const std::array<InnerNode<LEAF_NODE>, 8>& children,
													unsigned int depth)
	{
		if (isNodeCollapsible(children))
		{
			// Note: Can not assume that these are the same in this function
			node.logit = children[0].logit;
			prune(node, depth);
			return true;
		}

		double new_logit = getMaxChildLogit(children);
		bool new_contains_free = false;
		bool new_contains_unknown = false;
		for (const InnerNode<LEAF_NODE>& child : children)
		{
			if (containsFree(child))
			{
				new_contains_free = true;
			}
			if (containsUnknown(child))
			{
				new_contains_unknown = true;
			}
		}

		if (node.logit != new_logit || node.contains_free != new_contains_free ||
				node.contains_unknown != new_contains_unknown)
		{
			node.logit = new_logit;
			node.contains_free = new_contains_free;
			node.contains_unknown = new_contains_unknown;
			return true;
		}
		return false;
	}

	//
	// BBX
	//

	bool getIntersection(double d_1, double d_2, const Point3& p_1, const Point3& p_2,
											 Point3* hit) const
	{
		if (0 <= (d_1 * d_2))
		{
			return false;
		}
		*hit = p_1 + (p_2 - p_1) * (-d_1 / (d_2 - d_1));
		return true;
	}

	bool inBBX(const Point3& point, int axis, const Point3& bbx_min,
						 const Point3& bbx_max) const
	{
		if (0 == axis && point[2] > bbx_min[2] && point[2] < bbx_max[2] &&
				point[1] > bbx_min[1] && point[1] < bbx_max[1])
		{
			return true;
		}
		if (1 == axis && point[2] > bbx_min[2] && point[2] < bbx_max[2] &&
				point[0] > bbx_min[0] && point[0] < bbx_max[0])
		{
			return true;
		}
		if (2 == axis && point[0] > bbx_min[0] && point[0] < bbx_max[0] &&
				point[1] > bbx_min[1] && point[1] < bbx_max[1])
		{
			return true;
		}
		return false;
	}

	//
	// Checking state of node
	//

	bool isOccupied(const LEAF_NODE& node) const
	{
		return isOccupiedLog(node.logit);
	}

	bool isFree(const LEAF_NODE& node) const
	{
		return isFreeLog(node.logit);
	}

	bool isUnknown(const LEAF_NODE& node) const
	{
		return isUnknownLog(node.logit);
	}

	bool containsOccupied(const InnerNode<LEAF_NODE>& node) const
	{
		return isOccupied(static_cast<LEAF_NODE&>(node));
	}

	bool containsFree(const InnerNode<LEAF_NODE>& node) const
	{
		return node.contains_free;
	}

	bool containsUnknown(const InnerNode<LEAF_NODE>& node) const
	{
		return node.contains_unknown;
	}

	//
	// Create / delete children
	//

	bool createChildren(InnerNode<LEAF_NODE>& inner_node, unsigned int depth)
	{
		if (nullptr != inner_node.children)
		{
			return false;
		}

		if (1 == depth)
		{
			inner_node.children = new std::array<LEAF_NODE, 8>();
			num_leaf_nodes_ += 8;
			num_inner_leaf_nodes_ -= 1;
			num_inner_nodes_ += 1;
		}
		else
		{
			inner_node.children = new std::array<InnerNode<LEAF_NODE>, 8>();
			for (InnerNode<LEAF_NODE>& child : getInnerChildren(inner_node))
			{
				child.contains_free = isFree(child);
				child.contains_unknown = isUnknown(child);
			}
			num_inner_leaf_nodes_ += 7;  // Get 8 new and 1 is made into a inner node
			num_inner_nodes_ += 1;
		}
		inner_node.all_children_same = false;  // Chould this be here or in expand?

		return true;
	}

	bool expand(InnerNode<LEAF_NODE>& inner_node, unsigned int depth)
	{
		if (!inner_node.all_children_same)
		{
			return false;
		}

		createChildren(inner_node, depth);

		if (1 == depth)
		{
			for (LEAF_NODE& child : getLeafChildren(inner_node))
			{
				child.logit = inner_node.logit;
			}
		}
		else
		{
			for (InnerNode<LEAF_NODE>& child : getInnerChildren(inner_node))
			{
				child.logit = inner_node.logit;
				child.contains_free = inner_node.contains_free;
				child.contains_unknown = inner_node.contains_unknown;
				child.all_children_same = true;
			}
		}

		return true;
	}

	void deleteChildren(InnerNode<LEAF_NODE>& inner_node, unsigned int depth,
											bool manual_pruning = false)
	{
		inner_node.all_children_same = true;

		if (nullptr == inner_node.children ||
				(!manual_pruning && !automatic_pruning_enabled_))
		{
			return;
		}

		if (1 == depth)
		{
			delete &getLeafChildren(inner_node);
			num_leaf_nodes_ -= 8;
			num_inner_leaf_nodes_ += 1;
			num_inner_nodes_ -= 1;
		}
		else
		{
			std::array<InnerNode<LEAF_NODE>, 8>& children = getInnerChildren(inner_node);
			unsigned int child_depth = depth - 1;
			for (InnerNode<LEAF_NODE>& child : children)
			{
				deleteChildren(child, child_depth, manual_pruning);
			}
			delete &children;
			num_inner_leaf_nodes_ -=
					7;  // Remove 8 and 1 inner node is made into a inner leaf node
			num_inner_nodes_ -= 1;
		}
		inner_node.children = nullptr;
	}

	void prune(InnerNode<LEAF_NODE>& inner_node, unsigned int depth,
						 bool manual_pruning = false)
	{
		deleteChildren(inner_node, depth, manual_pruning);
		inner_node.contains_free = isFree(inner_node);
		inner_node.contains_unknown = isUnknown(inner_node);
	}

	//
	// Get children
	//

	inline std::array<LEAF_NODE, 8>&
	getLeafChildren(const InnerNode<LEAF_NODE>& inner_node) const
	{
		return *static_cast<std::array<LEAF_NODE, 8>*>(inner_node.children);
	}

	inline std::array<InnerNode<LEAF_NODE>, 8>&
	getInnerChildren(const InnerNode<LEAF_NODE>& inner_node) const
	{
		return *static_cast<std::array<InnerNode<LEAF_NODE>, 8>*>(inner_node.children);
	}

	//
	// Node collapsible
	//

	virtual bool isNodeCollapsible(const std::array<LEAF_NODE, 8>& children) const
	{
		for (int i = 1; i < 8; ++i)
		{
			// if (children[0].logit < children[i].logit - 0.1 ||
			// 		children[0].logit > children[i].logit + 0.1)
			if (children[0].logit != children[i].logit)
			{
				return false;
			}
		}
		return true;
	}

	virtual bool
	isNodeCollapsible(const std::array<InnerNode<LEAF_NODE>, 8>& children) const
	{
		if (isLeaf(children[0]))
		{
			for (int i = 1; i < 8; ++i)
			{
				if (children[0].logit != children[i].logit || !isLeaf(children[i]))
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
	// Clear
	//

	void clear(InnerNode<LEAF_NODE>& inner_node, unsigned int current_depth)
	{
		deleteChildren(inner_node, current_depth, true);
	}

	//
	// Checking for children
	//

	bool isLeaf(const InnerNode<LEAF_NODE>& node) const
	{
		return node.all_children_same;
	}

	bool hasChildren(const InnerNode<LEAF_NODE>& node) const
	{
		return !node.all_children_same;
	}

	//
	// Random functions
	//

	bool containsOnlySameType(const LEAF_NODE& node) const
	{
		return true;
	}

	bool containsOnlySameType(const InnerNode<LEAF_NODE>& node) const
	{
		if (isOccupied(node))
		{
			return !containsFree(node) && !containsUnknown(node);
		}
		else if (isUnknown(node))
		{
			return !containsFree(node);
		}
		return true;  // Is free and does only contain free children
	}

	void computeUpdate(const Point3& sensor_origin, const PointCloud& cloud,
										 double max_range)
	{
		// Source: A Faster Voxel Traversal Algorithm for Ray Tracing

		for (size_t i = 0; i < cloud.size(); ++i)
		{
			Point3 origin = sensor_origin;
			Point3 end = cloud[i] - origin;
			double distance = end.norm();
			Point3 dir = end / distance;
			if (0 <= max_range && distance > max_range)
			{
				end = origin + (dir * max_range);
			}
			else
			{
				end = cloud[i];
			}

			// Move origin and end to inside BBX
			if (!moveLineIntoBBX(origin, end))
			{
				// Line outside of BBX
				continue;
			}

			if (cloud[i] == end)
			{
				indices_[Code(coordToKey(end, 0))] = prob_hit_log_;
			}

			Key current;
			Key ending;

			std::array<int, 3> step;
			Point3 t_delta;
			Point3 t_max;

			computeRayInit(origin, end, dir, current, ending, step, t_delta, t_max);

			// Increment
			while (current != ending && t_max.min() <= distance)
			{
				indices_.try_emplace(current, prob_miss_log_);
				computeRayTakeStep(current, step, t_delta, t_max);
			}
		}
	}

	void computeUpdateDiscrete(const Point3& sensor_origin, const std::vector<Key>& current,
														 const KeyMap<std::vector<Key>>& discrete_map,
														 unsigned int n = 0)
	{
		// Source: A Faster Voxel Traversal Algorithm for Ray Tracing

		for (const auto& key : current)
		{
			Point3 origin = sensor_origin;
			Point3 end = keyToCoord(key) - sensor_origin;
			double distance = end.norm();
			Point3 dir = end / distance;
			end = origin + (dir * distance);

			if (0 == key.getDepth())
			{
				Key current;
				Key ending;

				std::array<int, 3> step;
				Point3 t_delta;
				Point3 t_max;

				computeRayInit(sensor_origin, end, dir, current, ending, step, t_delta, t_max,
											 key.getDepth());

				// Increment
				while (current != ending && t_max.min() <= distance)
				{
					indices_.try_emplace(current, prob_miss_log_);
					computeRayTakeStep(current, step, t_delta, t_max, key.getDepth());
				}
			}
			else
			{
				double node_size = getNodeSize(key.getDepth());
				int num_steps = (distance / node_size) - n;

				Point3 current = origin;
				Point3 last = current;
				Key current_key = coordToKey(current, key.getDepth());
				int step = 0;
				double value = prob_miss_log_ / double((2.0 * key.getDepth()) + 1);
				while (current_key != key && step <= num_steps)
				{
					last = current;
					indices_.try_emplace(current_key, value);
					current += (dir * node_size);
					current_key = coordToKey(current, key.getDepth());
					++step;
				}

				if (0 == n)
				{
					indices_.try_emplace(current_key, value);
				}
				else
				{
					computeUpdateDiscrete(last, discrete_map.at(key), discrete_map, n);
				}
			}
		}
	}

	void computeRayInit(const Point3& origin, const Point3& end,
											const Point3& direction_normalized, Key& current, Key& ending,
											std::array<int, 3>& step, Point3& t_delta, Point3& t_max,
											unsigned int depth = 0) const
	{
		current = coordToKey(origin, depth);
		ending = coordToKey(end, depth);

		if (current == ending)
		{
			return;
		}

		Point3 voxel_border = keyToCoord(current);

		for (unsigned int i = 0; i < 3; ++i)
		{
			if (0 < direction_normalized[i])
			{
				step[i] = 1;
			}
			else if (0 > direction_normalized[i])
			{
				step[i] = -1;
			}
			else
			{
				step[i] = 0;
			}

			if (0 != step[i])
			{
				t_delta[i] = getNodeSize(depth) / std::fabs(direction_normalized[i]);

				voxel_border[i] += (double)(step[i] * getNodeHalfSize(depth));
				t_max[i] = (voxel_border[i] - origin[i]) / direction_normalized[i];
			}
			else
			{
				t_delta[i] = std::numeric_limits<double>::max();
				t_max[i] = std::numeric_limits<double>::max();
			}
		}
	}

	inline void computeRayTakeStep(Key& current, const std::array<int, 3>& step,
																 const Point3& t_delta, Point3& t_max,
																 unsigned int depth = 0) const
	{
		size_t advance_dim = t_max.minElementIndex();
		current[advance_dim] += step[advance_dim] << depth;
		t_max[advance_dim] += t_delta[advance_dim];
	}

	double getMaxChildLogit(const std::array<InnerNode<LEAF_NODE>, 8>& children) const
	{
		double max = std::numeric_limits<double>::lowest();  // TODO: Check this one, maybe
																											 // should be lowest()?
		for (const LEAF_NODE& child : children)
		{
			if (max < child.logit)
			{
				max = child.logit;
			}
		}
		return max;
	}

	double getMaxChildLogit(const std::array<LEAF_NODE, 8>& children) const
	{
		double max = children[0].logit;  // std::numeric_limits<double>::lowest();  // TODO:
																		// Check this one, maybe should be lowest()?
		for (const LEAF_NODE& child : children)
		{
			if (max < child.logit)
			{
				max = child.logit;
			}
		}
		return max;
	}

	double getMeanChildLogit(const std::array<InnerNode<LEAF_NODE>, 8>& children) const
	{
		double mean = 0;
		int num = 0;
		for (const LEAF_NODE& child : children)
		{
			mean += probability(child.logit);
			++num;
		}
		if (0 < num)
		{
			mean /= num;
		}
		return logit(mean);
	}

	double getMeanChildLogit(const std::array<LEAF_NODE, 8>& children) const
	{
		double mean = 0;
		int num = 0;
		for (const LEAF_NODE& child : children)
		{
			mean += probability(child.logit);
			++num;
		}
		if (0 < num)
		{
			mean /= num;
		}
		return logit(mean);
	}

	//
	// Read/write
	//

	virtual bool binarySupport() const
	{
		return false;
	}

	static std::pair<std::string, bool> readFirstLineHeader(std::istream& s)
	{
		// check if first line valid:
		std::string line;
		std::getline(s, line);
		if (0 == line.compare(0, FILE_HEADER.length(), FILE_HEADER))
		{
			// UFOMap
			return std::make_pair("ufomap", false);
		}
		else if (0 == line.compare(0, FILE_HEADER_OCTOMAP.length(), FILE_HEADER_OCTOMAP))
		{
			// OctoMap
			return std::make_pair("octomap", false);
		}
		else if (0 == line.compare(0, BINARY_FILE_HEADER.length(), BINARY_FILE_HEADER))
		{
			// UFOMap binary
			return std::make_pair("ufomap", true);
		}
		else if (0 == line.compare(0, BINARY_FILE_HEADER_OCTOMAP.length(),
															 BINARY_FILE_HEADER_OCTOMAP))
		{
			// OctoMap binary
			return std::make_pair("octomap", true);
		}
		else
		{
			// Error "First line of OcTree file header does not start with \"" << fileHeader
			return std::make_pair("", false);
		}
	}

	bool readHeader(std::istream& s, std::string& id, size_t& size, double& res,
									unsigned int& depth_levels, double& occupancy_thres, double& free_thres,
									bool& compressed, int& data_size, int& compressed_data_size,
									bool is_ufomap = true)
	{
		id = "";
		size = 0;
		res = 0.0;
		depth_levels = 0;
		occupancy_thres = 0.5;
		free_thres = 0.5;
		compressed = false;
		data_size = 0;
		compressed_data_size = 0;

		std::string token;
		bool header_read = false;
		while (s.good() && !header_read)
		{
			s >> token;
			if ("data" == token)
			{
				header_read = true;
				// skip forward until end of line:
				char c;
				do
				{
					c = s.get();
				} while (s.good() && (c != '\n'));
			}
			else if (0 == token.compare(0, 1, "#"))
			{
				// comment line, skip forward until end of line:
				char c;
				do
				{
					c = s.get();
				} while (s.good() && (c != '\n'));
			}
			else if ("id" == token)
			{
				s >> id;
			}
			else if ("res" == token)
			{
				s >> res;
			}
			else if ("size" == token)
			{
				s >> size;
			}
			else if ("depth_levels" == token)
			{
				s >> depth_levels;
			}
			else if ("occupancy_thres" == token)
			{
				s >> occupancy_thres;
			}
			else if ("free_thres" == token)
			{
				s >> free_thres;
			}
			else if ("compressed" == token)
			{
				s >> compressed;
			}
			else if ("data_size" == token)
			{
				s >> data_size;
			}
			else if ("compressed_data_size" == token)
			{
				s >> compressed_data_size;
			}
			else
			{
				// Other token
				char c;
				do
				{
					c = s.get();
				} while (s.good() && (c != '\n'));
			}
		}

		if (!header_read)
		{
			return false;
		}

		if ("" == id)
		{
			return false;
		}

		if (0.0 >= res)
		{
			return false;
		}

		// TODO: What to do?
		if (0.0 > occupancy_thres)
		{
			return false;
		}

		// TODO: What to do?
		if (0.0 > free_thres)
		{
			return false;
		}

		if (is_ufomap)
		{
			if (0 == depth_levels)
			{
				// Missing depth levels token
				return false;
			}
		}
		else
		{
			depth_levels = 16;
		}

		if ((is_ufomap && getTreeType() != id) || (!is_ufomap && getTreeTypeOctomap() != id))
		{
			// Wrong tree type
			return false;
		}

		return true;
	}

	int getDataSize(std::istream& s) const
	{
		s.seekg(0, s.end);
		const int size = s.tellg();
		s.seekg(0, s.beg);
		return size;
	}

	bool readNodesRecurs(std::istream& s, InnerNode<LEAF_NODE>& node,
											 unsigned int current_depth, double occupancy_thres_log,
											 double free_thres_log, bool from_octomap = false)
	{
		static_cast<LEAF_NODE&>(node).readData(s, occupancy_thres_log, free_thres_log,
																					 from_octomap);
		node.all_children_same = true;

		char children_char;
		s.read((char*)&children_char, sizeof(char));
		std::bitset<8> children((unsigned long long)children_char);

		if (children.any())
		{
			expand(node, current_depth);

			// FIXME: Check so correct order

			if (1 == current_depth)
			{
				for (LEAF_NODE& child : *static_cast<std::array<LEAF_NODE, 8>*>(node.children))
				{
					child.readData(s, occupancy_thres_log, free_thres_log, from_octomap);
				}
			}
			else
			{
				for (InnerNode<LEAF_NODE>& child :
						 *static_cast<std::array<InnerNode<LEAF_NODE>, 8>*>(node.children))
				{
					readNodesRecurs(s, child, current_depth - 1, occupancy_thres_log,
													free_thres_log, from_octomap);
				}
			}
		}

		updateNode(node, current_depth);  // To set indicators

		return true;
	}

	virtual bool readBinaryNodesRecurs(std::istream& s, InnerNode<LEAF_NODE>& node,
																		 unsigned int current_depth,
																		 double occupancy_thres_log, double free_thres_log,
																		 bool from_octomap = false)
	{
		return false;
	}

	bool writeNodesRecurs(std::ostream& s, const InnerNode<LEAF_NODE>& node,
												unsigned int current_depth, bool to_octomap = false) const
	{
		static_cast<const LEAF_NODE&>(node).writeData(s, occupancy_thres_log_,
																									free_thres_log_, to_octomap);

		// 1 bit for each children; 0: empty, 1: allocated
		std::bitset<8> children;
		if (hasChildren(node))
		{
			children.flip();
		}

		char children_char = (char)children.to_ulong();
		s.write((char*)&children_char, sizeof(char));

		if (children.any())
		{
			// Recursively write children
			if (1 == current_depth)
			{
				for (const LEAF_NODE& child :
						 *static_cast<std::array<LEAF_NODE, 8>*>(node.children))
				{
					child.writeData(s, occupancy_thres_log_, free_thres_log_, to_octomap);
				}
			}
			else
			{
				for (const InnerNode<LEAF_NODE>& child :
						 *static_cast<std::array<InnerNode<LEAF_NODE>, 8>*>(node.children))
				{
					writeNodesRecurs(s, child, current_depth - 1, to_octomap);
				}
			}
		}

		return true;
	}

	virtual bool writeBinaryNodesRecurs(std::ostream& s, const InnerNode<LEAF_NODE>& node,
																			unsigned int current_depth,
																			bool to_octomap = false) const
	{
		return false;
	}

protected:
	double resolution_;           // The voxel size of the leaf nodes
	double resolution_factor_;    // Reciprocal of the resolution
	unsigned int depth_levels_;  // The maximum depth of the octree
	unsigned int max_value_;     // The maximum coordinate value the octree can store

	// Sensor model
	double occupancy_thres_log_;     // Threshold for occupancy
	double free_thres_log_;          // Threshold for free
	double prob_hit_log_;            // Logodds probability of hit
	double prob_miss_log_;           // Logodds probability of miss
	double clamping_thres_min_log_;  // Min logodds value
	double clamping_thres_max_log_;  // Max logodds value

	// Bounding box
	bool bbx_limit_enabled_ = false;  // Use bounding box for queries?
	Point3 bbx_min_;                  // Minimum coordinate for bounding box
	Point3 bbx_max_;                  // Maximum coordinate for bounding box
	Key bbx_min_key_;                 // Minimum key for bounding box
	Key bbx_max_key_;                 // Maximum key for bounding box

	// Change detection
	bool change_detection_enabled_ = false;  // Flag if change detection is enabled or not
	CodeSet changed_codes_;                  // Set of codes that have changed since last
																					 // resetChangeDetection

	// The root of the octree
	InnerNode<LEAF_NODE> root_;
	std::vector<double> nodes_sizes_;
	std::vector<double> nodes_half_sizes_;

	// Automatic pruning
	bool automatic_pruning_enabled_ = true;

	// Memory
	size_t num_inner_nodes_ = 0;
	size_t num_inner_leaf_nodes_ = 1;  // The root node
	size_t num_leaf_nodes_ = 0;

	// Defined here for speedup
	std::array<std::future<void>, 2> insert_async_handler_;
	CodeMap<double> indices_;         // Used in insertPointCloud
	CodeMap<double> indices_buffer_;  // Used in insertPointCloud

	// File headers
	inline static const std::string FILE_HEADER = "# UFOMap octree file";
	inline static const std::string BINARY_FILE_HEADER = "# UFOMap octree binary file";
	// To be able to read/write OctoMap files
	inline static const std::string FILE_HEADER_OCTOMAP = "# Octomap OcTree file";
	inline static const std::string BINARY_FILE_HEADER_OCTOMAP = "# Octomap OcTree binary "
																															 "file";
};
}  // namespace ufomap

// #include <ufomap/octree_base.hxx>  // The implementation of this

#endif  // UFOMAP_OCTREE_BASE_H