#ifndef UFOMAP_GEOMETRY_BOUDING_VOLUME_H
#define UFOMAP_GEOMETRY_BOUDING_VOLUME_H

#include <ufomap/geometry/collision_checks.h>
#include <ufomap/geometry/types.h>
#include <ufomap/math/pose6.h>
#include <ufomap/math/quaternion.h>
#include <ufomap/math/vector3.h>

#include <exception>

namespace ufomap_geometry
{
class BoundingVolume
{
public:
	void add(const BoundingVar& bv)
	{
		bounding_volume_.push_back(bv);
	}

	size_t size() const
	{
		return bounding_volume_.size();
	}

	bool empty() const
	{
		return bounding_volume_.empty();
	}

	bool intersects(const BoundingVar& bv) const;

	bool intersects(const BoundingVolume& other) const;

	std::vector<BoundingVar>::iterator begin()
	{
		return bounding_volume_.begin();
	}

	std::vector<BoundingVar>::const_iterator begin() const
	{
		return bounding_volume_.begin();
	}

	std::vector<BoundingVar>::const_iterator cbegin() const
	{
		return bounding_volume_.cbegin();
	}

	std::vector<BoundingVar>::iterator end()
	{
		return bounding_volume_.end();
	}

	std::vector<BoundingVar>::const_iterator end() const
	{
		return bounding_volume_.end();
	}

	std::vector<BoundingVar>::const_iterator cend() const
	{
		return bounding_volume_.cend();
	}

	std::vector<BoundingVar>::reverse_iterator rbegin()
	{
		return bounding_volume_.rbegin();
	}

	std::vector<BoundingVar>::const_reverse_iterator rbegin() const
	{
		return bounding_volume_.rbegin();
	}

	std::vector<BoundingVar>::const_reverse_iterator crbegin() const
	{
		return bounding_volume_.crbegin();
	}

	std::vector<BoundingVar>::reverse_iterator rend()
	{
		return bounding_volume_.rend();
	}

	std::vector<BoundingVar>::const_reverse_iterator rend() const
	{
		return bounding_volume_.rend();
	}

	std::vector<BoundingVar>::const_reverse_iterator crend() const
	{
		return bounding_volume_.crend();
	}

	void translate(const ufomap_math::Vector3& translation)
	{
		throw std::logic_error("Function not yet implemented");
		// TODO: Implement
	}

	void rotate(const ufomap_math::Quaternion& rotation)
	{
		throw std::logic_error("Function not yet implemented");
		// TODO: Implement
	}

	void rotate(const ufomap_math::Vector3& rotation)
	{
		throw std::logic_error("Function not yet implemented");
		// TODO: Implement
	}

	void transform(const ufomap_math::Pose6& transform)
	{
		throw std::logic_error("Function not yet implemented");
		// TODO: Implement
	}

private:
	std::vector<BoundingVar> bounding_volume_;
};
}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_BOUDING_VOLUME_H