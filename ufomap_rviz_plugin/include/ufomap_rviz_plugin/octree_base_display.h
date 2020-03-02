#ifndef UFOMAP_RVIZ_PLUGIN_OCTREE_BASE_DISPLAY_H
#define UFOMAP_RVIZ_PLUGIN_OCTREE_BASE_DISPLAY_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/ogre_helpers/point_cloud.h>
#endif  // Q_MOC_RUN

#include <message_filters/subscriber.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/visualization_manager.h>

#include <ufomap_msgs/Ufomap.h>

#include <memory>
#include <mutex>
#include <unordered_map>

namespace ufomap_rviz_plugin
{
class OctreeBaseDisplay : public rviz::Display
{
	Q_OBJECT
public:
	enum OctreeVoxelType
	{
		UFOMAP_OCCUPIED,
		UFOMAP_FREE,
		UFOMAP_UNKNOWN,
	};

	enum OctreeVoxelStyle
	{
		UFOMAP_POINTS_STYLE,
		UFOMAP_SQUARES_STYLE,
		UFOMAP_FLAT_SQUARES_STYLE,
		UFOMAP_SPHERES_STYLE,
		UFOMAP_BOXES_STYLE,
	};

	virtual ~OctreeBaseDisplay()
	{
		unsubscribe();

		clouds_.clear();

		if (scene_node_)
		{
			scene_node_->detachAllObjects();
		}
	}

	// Overrides from rviz::Display
	virtual void onInitialize() override
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
				clouds_[type][d].setRenderMode(rviz::PointCloud::RM_FLAT_SQUARES);
				clouds_[type][d].setCastShadows(false);
				scene_node_->attachObject(&clouds_[type][d]);
			}
		}
	}

	// virtual void update(float wall_dt, float ros_dt);

	virtual void reset() override
	{
		clear();
		num_messages_received_ = 0;
		setStatus(rviz::StatusProperty::Ok, "Messages",
							QString("0 ufomap messages received"));
	}

protected Q_SLOTS:
	void updateQueueSize()
	{
		queue_size_ = queue_size_property_->getInt();
		subscribe();
	}

	void updateTopic()
	{
		unsubscribe();
		reset();
		subscribe();
		context_->queueRender();
	}

	void updateTreeDepth()
	{
		// TODO: Implement
	}

	void updateOctreeRenderMode()
	{
		should_update_ = true;
	}

	void updateOctreeRenderStyle()
	{
		for (const OctreeVoxelType& type : render_mode_.keys())
		{
			for (auto& cloud : clouds_[type])
			{
				switch (render_mode_[type]->getOptionInt())
				{
					case UFOMAP_POINTS_STYLE:
						cloud.setRenderMode(rviz::PointCloud::RM_POINTS);
						break;
					case UFOMAP_SQUARES_STYLE:
						cloud.setRenderMode(rviz::PointCloud::RM_SQUARES);
						break;
					case UFOMAP_FLAT_SQUARES_STYLE:
						cloud.setRenderMode(rviz::PointCloud::RM_FLAT_SQUARES);
						break;
					case UFOMAP_SPHERES_STYLE:
						cloud.setRenderMode(rviz::PointCloud::RM_SPHERES);
						break;
					case UFOMAP_BOXES_STYLE:
						cloud.setRenderMode(rviz::PointCloud::RM_BOXES);
						break;
				}
			}
		}
		should_update_ = true;
	}

	virtual void updateOctreeColorMode() = 0;

	void updateAlpha()
	{
		should_update_ = true;
	}

	void updateDepth()
	{
		should_update_ = true;
	}

	void updateBBX()
	{
		should_update_ = true;
	}

protected:
	OctreeBaseDisplay()
		: rviz::Display(), should_update_(false), num_messages_received_(0), queue_size_(10)
	{
	}

	// Overrides from rviz::Display
	virtual void onEnable() override
	{
		scene_node_->setVisible(true);
		subscribe();
	}

	virtual void onDisable() override
	{
		scene_node_->setVisible(false);
		unsubscribe();
		clear();
	}

	void subscribe()
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

	void unsubscribe()
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

	virtual void octreeCallback(const ufomap_msgs::Ufomap::ConstPtr& msg) = 0;

	void setColor(float value, float min_value, float max_value, float color_factor,
								rviz::PointCloud::Point& point) const
	{
		// Copied from OctoMap
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
		{
			f = 1 - f;
		}
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

	void clear()
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

	virtual bool updateFromTF()
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

	virtual bool checkType(const std::string& type) const = 0;

	static std::string getStrVoxelType(const OctreeVoxelType& type)
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

protected:
	unsigned int queue_size_;
	unsigned int num_messages_received_;

	std::mutex mutex_;

	std::shared_ptr<message_filters::Subscriber<ufomap_msgs::Ufomap>> sub_;

	// Plugin properties
	rviz::IntProperty* queue_size_property_;
	rviz::RosTopicProperty* topic_property_;
	QHash<OctreeVoxelType, rviz::BoolProperty*> render_type_;
	rviz::Property* render_category_property_;
	QHash<OctreeVoxelType, rviz::EnumProperty*> render_mode_;
	QHash<OctreeVoxelType, rviz::EnumProperty*> coloring_property_;
	QHash<OctreeVoxelType, rviz::ColorProperty*> color_property_;
	QHash<OctreeVoxelType, rviz::FloatProperty*> color_factor_property_;
	QHash<OctreeVoxelType, rviz::FloatProperty*> alpha_property_;
	rviz::IntProperty* depth_property_;
	rviz::BoolProperty* use_bbx_property_;
	rviz::VectorProperty* min_bbx_property_;
	rviz::VectorProperty* max_bbx_property_;

	bool should_update_;

	// Point clouds
	QHash<OctreeVoxelType, std::vector<rviz::PointCloud>> clouds_;
	std_msgs::Header header_;

	inline static const unsigned int MAX_OCTREE_DEPTH_LEVELS_ = 21;  // What should this be?
};

}  // namespace ufomap_rviz_plugin

#endif  // UFOMAP_RVIZ_PLUGIN_OCTREE_BASE_DISPLAY_H