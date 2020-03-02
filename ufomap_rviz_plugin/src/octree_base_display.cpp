#include <ufomap_rviz_plugin/octree_base_display.h>

#include <rviz/properties/status_property.h>

namespace ufomap_rviz_plugin
{
OctreeBaseDisplay::OctreeBaseDisplay()
	: rviz::Display(), should_update_(false), num_messages_received_(0), queue_size_(10)
{
}

OctreeBaseDisplay::~OctreeBaseDisplay()
{
	unsubscribe();

	clouds_.clear();

	if (scene_node_)
	{
		scene_node_->detachAllObjects();
	}
}

void OctreeBaseDisplay::onInitialize()
{
	const std::lock_guard<std::mutex> lock(mutex_);

	for (const OctreeVoxelType& type : { UFOMAP_OCCUPIED, UFOMAP_FREE, UFOMAP_UNKNOWN })
	{
		clouds_[type].resize(MAX_OCTREE_DEPTH_LEVELS_);

		for (unsigned int d = 0; d < MAX_OCTREE_DEPTH_LEVELS_; ++d)
		{
			std::stringstream sname;
			sname << getStrVoxelType(type) << " point cloud depth " << d;
			clouds_[type][d].setName(sname.str());
			clouds_[type][d].setRenderMode(rviz::PointCloud::RM_BOXES);
			clouds_[type][d].setCastShadows(false);
			scene_node_->attachObject(&clouds_[type][d]);
		}
	}
}

void OctreeBaseDisplay::reset()
{
	clear();
	num_messages_received_ = 0;
	setStatus(rviz::StatusProperty::Ok, "Messages", QString("0 ufomap messages received"));
}

void OctreeBaseDisplay::updateQueueSize()
{
	queue_size_ = queue_size_property_->getInt();
	subscribe();
}

void OctreeBaseDisplay::updateTopic()
{
	unsubscribe();
	reset();
	subscribe();
	context_->queueRender();
}

void OctreeBaseDisplay::updateTreeDepth()
{
	// TOOD: Implement
}

void OctreeBaseDisplay::updateOctreeRenderMode()
{
	updateTopic();
}

void OctreeBaseDisplay::updateOctreeColorMode()
{
	updateTopic();
}

void OctreeBaseDisplay::updateAlpha()
{
	should_update_ = true;
}

void OctreeBaseDisplay::updateDepth()
{
	should_update_ = true;
}

void OctreeBaseDisplay::updateBBX()
{
	should_update_ = true;
}

void OctreeBaseDisplay::onEnable()
{
	scene_node_->setVisible(true);
	subscribe();
}

void OctreeBaseDisplay::onDisable()
{
	scene_node_->setVisible(false);
	unsubscribe();
	clear();
}

void OctreeBaseDisplay::subscribe()
{
	if (!isEnabled())
	{
		return;
	}

	try
	{
		unsubscribe();

		const std::string& topic = topic_property_->getStdString();

		if (!topic.empty())
		{
			sub_.reset(new message_filters::Subscriber<ufomap_msgs::Ufomap>());

			sub_->subscribe(threaded_nh_, topic, queue_size_);
			sub_->registerCallback(boost::bind(&OctreeBaseDisplay::octreeCallback, this, _1));
		}
	}
	catch (ros::Exception& e)
	{
		setStatus(rviz::StatusProperty::Error, "Topic",
							(std::string("Error subscribing: ") + e.what()).c_str());
	}
}

void OctreeBaseDisplay::unsubscribe()
{
	clear();

	try
	{
		sub_.reset();
	}
	catch (ros::Exception& e)
	{
		setStatus(rviz::StatusProperty::Error, "Topic",
							(std::string("Error unsubscribing: ") + e.what()).c_str());
	}
}

void OctreeBaseDisplay::octreeCallback(const ufomap_msgs::Ufomap::ConstPtr& msg)
{
	++num_messages_received_;
	setStatus(rviz::StatusProperty::Ok, "Messages",
						QString::number(num_messages_received_) + " ufomap messages received");
	setStatusStd(rviz::StatusProperty::Ok, "Type", msg->id.c_str());
	if (!checkType(msg->id))
	{
		setStatusStd(rviz::StatusProperty::Error, "Message",
								 "Wrong ufomap type. Use a different display type.");
	}

	header_ = msg->header;
	if (!updateFromTF())
	{
		std::stringstream ss;
		ss << "Failed to transform from frame [" << header_.frame_id << "] to frame ["
			 << context_->getFrameManager()->getFixedFrame() << "]";
		setStatusStd(rviz::StatusProperty::Error, "Message", ss.str());
		return;
	}

	const std::lock_guard<std::mutex> lock(mutex_);

	if (!ufomap_msgs::msgToMap(*msg, ufomap_))
	{
		setStatusStd(rviz::StatusProperty::Error, "Message", "Could not create octree.");
	}

	for (const auto& type : depth_property_.keys())
	{
		depth_property_[type]->setMax(ufomap_.getTreeDepthLevels());
	}

	should_update_ = true;

	// TODO: Do something with tree depth
}

void OctreeBaseDisplay::setColor(float value, float min_value, float max_value,
																 float color_factor, rviz::PointCloud::Point& point) const
{
	int i;
	double m, n, f;

	double s = 1.0;
	double v = 1.0;

	double h =
			(1.0 -
			 std::min(std::max((value - min_value) / (max_value - min_value), 0.0f), 1.0f)) *
			color_factor;

	h -= floor(h);
	h *= 6;
	i = floor(h);
	f = h - i;
	if (!(i & 1))
		f = 1 - f;  // if i is even
	m = v * (1 - s);
	n = v * (1 - s * f);

	switch (i)
	{
		case 6:
		case 0:
			point.setColor(v, n, m);
			break;
		case 1:
			point.setColor(n, v, m);
			break;
		case 2:
			point.setColor(m, v, n);
			break;
		case 3:
			point.setColor(m, n, v);
			break;
		case 4:
			point.setColor(n, m, v);
			break;
		case 5:
			point.setColor(v, m, n);
			break;
		default:
			point.setColor(1, 0.5, 0.5);
			break;
	}
}

void OctreeBaseDisplay::clear()
{
	const std::lock_guard<std::mutex> lock(mutex_);

	for (auto& type : clouds_.keys())
	{
		for (auto& cloud : clouds_[type])
		{
			cloud.clear();
		}
	}
}

bool OctreeBaseDisplay::updateFromTF()
{
	Ogre::Vector3 position;
	Ogre::Quaternion orientation;
	if (context_->getFrameManager()->getTransform(header_, position, orientation))
	{
		scene_node_->setOrientation(orientation);
		scene_node_->setPosition(position);
		return true;
	}
	return false;
}

std::string OctreeBaseDisplay::getStrVoxelType(const OctreeVoxelType& type)
{
	switch (type)
	{
		case UFOMAP_OCCUPIED:
			return "Occupied";
		case UFOMAP_FREE:
			return "Free";
		case UFOMAP_UNKNOWN:
			return "Unknown";
	}
}

}  // namespace ufomap_rviz_plugin