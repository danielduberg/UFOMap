#include <ufomap_mapping/server_nodelet.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ufomap_mapping::UFOMapServerNodelet, nodelet::Nodelet)

namespace ufomap_mapping
{
void UFOMapServerNodelet::onInit()
{
	ros::NodeHandle& nh = getNodeHandle();
	ros::NodeHandle& nh_priv = getPrivateNodeHandle();

	if (nh_priv.param("multithreaded", false))
	{
		nh = getMTNodeHandle();
		nh_priv = getMTPrivateNodeHandle();
	}

	server_ = std::make_shared<UFOMapServer>(nh, nh_priv);
}
}  // namespace ufomap_mapping
