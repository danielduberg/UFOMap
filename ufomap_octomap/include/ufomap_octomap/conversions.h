#ifndef UFOMAP_OCTOMAP_CONVERSIONS_H
#define UFOMAP_OCTOMAP_CONVERSIONS_H

#include <ufomap/ufomap.h>

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>

namespace ufomap
{
void toUfomap(const octomap::OcTree& map_in, ufomap::Octree& map_out);

void toUfomap(const octomap::ColorOcTree& map_in, ufomap::OctreeRGB& map_out);

void fromUfomap(const ufomap::Octree& map_in, octomap::OcTree& map_out);

void fromUfomap(const ufomap::OctreeRGB& map_in, octomap::ColorOcTree& map_out);
}  // namespace ufomap

#endif  // UFOMAP_OCTOMAP_CONVERSIONS_H