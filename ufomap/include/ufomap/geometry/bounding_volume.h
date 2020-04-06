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
	template <typename TYPE>
	void add(const TYPE& bv)
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

	template <typename TYPE>
	bool intersects(const TYPE& other) const
	{
		for (const BoundingVar& bv : bounding_volume_)
		{
			if (std::visit([other](auto&& arg) -> bool { return intersects(other, arg); }, bv))
			{
				return true;
			}
		}
	}

	bool intersects(const BoundingVolume& other) const
	{
		for (const BoundingVar& other_bv : other.bounding_volume_)
		{
			if (std::visit([&](auto&& arg) -> bool { return intersects(arg); }, other_bv))
			{
				return true;
			}
		}
		return false;
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