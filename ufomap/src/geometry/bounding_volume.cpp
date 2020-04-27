#include <ufomap/geometry/bounding_volume.h>

namespace ufomap_geometry
{
bool BoundingVolume::intersects(const BoundingVar& bv) const
{
	for (const BoundingVar& bv_2 : bounding_volume_)
	{
		if (std::visit([](auto&& arg_1, auto&& arg_2)
											 -> bool { return ufomap_geometry::intersects(arg_1, arg_2); },
									 bv, bv_2))
		{
			return true;
		}
	}
	return false;
}

bool BoundingVolume::intersects(const BoundingVolume& other) const
{
	for (const BoundingVar& other_bv : other.bounding_volume_)
	{
		if (intersects(other_bv))
		{
			return true;
		}
	}
	return false;
}
}  // namespace ufomap_geometry