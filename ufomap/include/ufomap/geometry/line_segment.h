#ifndef UFOMAP_GEOMETRY_LINE_SEGMENT_H
#define UFOMAP_GEOMETRY_LINE_SEGMENT_H

#include <ufomap/math/vector3.h>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct LineSegment
{
	Vector3 start;
	Vector3 end;

	inline LineSegment()
	{
	}

	inline LineSegment(const Vector3& start, const Vector3& end) : start(start), end(end)
	{
	}
};
}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_LINE_SEGMENT_H