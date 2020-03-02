#ifndef UFOMAP_RVIZ_PLUGIN_OCTREE_DISPLAY_H
#define UFOMAP_RVIZ_PLUGIN_OCTREE_DISPLAY_H

#include <ufomap_rviz_plugin/octree_base_display.h>

#include <ufomap/octree.h>

namespace ufomap_rviz_plugin
{
class OctreeDisplay : public OctreeBaseDisplay
{
	Q_OBJECT
public:
	OctreeDisplay();

	virtual ~OctreeDisplay();

	virtual void update(float wall_dt, float ros_dt) override;

protected Q_SLOTS:
	virtual void updateOctreeColorMode() override;

protected:
	virtual void octreeCallback(const ufomap_msgs::Ufomap::ConstPtr& msg) override;

	void colorPoint(rviz::PointCloud::Point& point,
									const ufomap::Point3& min_value, const ufomap::Point3& max_value,
									float probability, OctreeVoxelType type) const;

	virtual bool checkType(const std::string& type) const override;

private:
	ufomap::Octree ufomap_;
};

}  // namespace ufomap_rviz_plugin

#endif  // UFOMAP_RVIZ_PLUGIN_OCTREE_DISPLAY_H