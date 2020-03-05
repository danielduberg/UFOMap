#ifndef UFOMAP_OCTREE_H
#define UFOMAP_OCTREE_H

#include <ufomap/node.h>
#include <ufomap/octree_base.h>
#include <ufomap/types.h>

namespace ufomap
{
class Octree : public OctreeBase<OccupancyNode>
{
public:
	Octree(float resolution = 0.1, unsigned int depth_levels = 16,
				 bool automatic_pruning = true, float occupancy_thres = 0.5,
				 float free_thres = 0.5, float prob_hit = 0.7, float prob_miss = 0.4,
				 float clamping_thres_min = 0.1192, float clamping_thres_max = 0.971);

	Octree(const std::string& filename);

	Octree(const Octree& other);

	virtual ~Octree()
	{
	}

	//
	// Tree type
	//

	virtual std::string getTreeType() const override
	{
		return "Octree";
	}

	virtual std::string getTreeTypeOctomap() const override
	{
		return "OcTree";
	}

	//
	// Read/write
	//

	// static std::pair<std::string, std::string> getBinaryType(const std::string& filename)
	// {
	// 	std::ifstream file(filename.c_str(), std::ios_base::in | std::ios_base::binary);
	// 	if (!file.is_open())
	// 	{
	// 		// Error
	// 		return std::make_pair("", "");
	// 	}
	// 	// TODO: check is_good of finished stream, warn?
	// 	return getBinaryType(file);
	// }

	// static std::pair<std::string, std::string> getBinaryType(std::ifstream& s)
	// {
	// 	std::istream::pos_type s_pos = s.tellg();

	// 	std::string type = readBinaryFirstLineHeader(s);
	// 	if ("" == type)
	// 	{
	// 		s.clear();
	// 		s.seekg(s_pos);
	// 		return std::make_pair("", "");
	// 	}

	// 	bool is_ufomap = "ufomap" == type;

	// 	std::string id;
	// 	size_t size;
	// 	float res;
	// 	unsigned int depth_levels;
	// 	if (!readHeader(s, id, size, res, depth_levels, is_ufomap))
	// 	{
	// 		s.clear();
	// 		s.seekg(s_pos);
	// 		return std::make_pair("", "");
	// 	}

	// 	s.clear();
	// 	s.seekg(s_pos);
	// 	return std::make_pair(type, id);
	// }

protected:
	//
	// Read/write
	//

	virtual bool binarySupport() const override
	{
		return true;
	}

	virtual bool readBinaryNodesRecurs(std::istream& s, InnerNode<OccupancyNode>& node,
																		 unsigned int current_depth,
																		 bool from_octomap = false) override;

	virtual bool writeBinaryNodesRecurs(std::ostream& s,
																			const InnerNode<OccupancyNode>& node,
																			unsigned int current_depth,
																			bool to_octomap = false) const override;
};
}  // namespace ufomap

#endif  // UFOMAP_OCTREE_H