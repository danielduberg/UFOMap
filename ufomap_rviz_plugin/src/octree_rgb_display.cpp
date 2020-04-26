#include <rviz/properties/ros_topic_property.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <ufomap_rviz_plugin/octree_rgb_display.h>

#include <numeric>

namespace ufomap_rviz_plugin
{
enum OctreeVoxelColorMode
{
	UFOMAP_VOXEL_COLOR,
	UFOMAP_X_AXIS_COLOR,
	UFOMAP_Y_AXIS_COLOR,
	UFOMAP_Z_AXIS_COLOR,
	UFOMAP_PROBABLILTY_COLOR,
	UFOMAP_FIXED_COLOR,
};

OctreeRGBDisplay::OctreeRGBDisplay() : OctreeBaseDisplay()
{
	topic_property_ = new rviz::RosTopicProperty(
			"Ufomap Topic", "",
			QString::fromStdString(ros::message_traits::datatype<ufomap_msgs::Ufomap>()),
			"ufomap_msgs::Ufomap topic to subscribe to", this, SLOT(updateTopic()));

	queue_size_property_ = new rviz::IntProperty("Queue size", queue_size_,
																							 "Set the size of the incoming message "
																							 "queue",
																							 this, SLOT(updateQueueSize()));
	queue_size_property_->setMin(1);

	render_category_property_ = new rviz::Property("Voxel Rendering", QVariant(), "", this);
	for (const OctreeVoxelType& type : { UFOMAP_OCCUPIED, UFOMAP_FREE, UFOMAP_UNKNOWN })
	{
		QString default_coloring;
		QColor default_color;
		float default_alpha;
		switch (type)
		{
			case UFOMAP_OCCUPIED:
				default_coloring = "Voxel Color";
				default_color = Qt::blue;
				default_alpha = 1.0;
				break;
			case UFOMAP_FREE:
				default_coloring = "Fixed";
				default_color = Qt::green;
				default_alpha = 0.03;
				break;
			case UFOMAP_UNKNOWN:
				default_coloring = "Fixed";
				default_color = Qt::white;
				default_alpha = 0.03;
		}

		QString type_str(getStrVoxelType(type).c_str());

		auto it = render_type_.insert(
				type, new rviz::BoolProperty(type_str, UFOMAP_OCCUPIED == type, "",
																		 render_category_property_,
																		 SLOT(updateOctreeRenderMode()), this));
		it.value()->setDisableChildrenIfFalse(true);

		auto render_mode_it = render_mode_.insert(
				type,
				new rviz::EnumProperty("Style", "Flat squares", "Select voxel rendering style",
															 it.value(), SLOT(updateOctreeRenderStyle()), this));
		render_mode_it.value()->addOption("Points", UFOMAP_POINTS_STYLE);
		render_mode_it.value()->addOption("Squares", UFOMAP_SQUARES_STYLE);
		render_mode_it.value()->addOption("Flat squares", UFOMAP_FLAT_SQUARES_STYLE);
		render_mode_it.value()->addOption("Spheres", UFOMAP_SPHERES_STYLE);
		render_mode_it.value()->addOption("Boxes", UFOMAP_BOXES_STYLE);

		auto coloring_it = coloring_property_.insert(
				type,
				new rviz::EnumProperty("Coloring", default_coloring, "Select voxel coloring mode",
															 it.value(), SLOT(updateOctreeColorMode()), this));
		if (UFOMAP_OCCUPIED == type)
		{
			coloring_it.value()->addOption("Voxel Color", UFOMAP_OCCUPIED);
		}
		coloring_it.value()->addOption("X-Axis", UFOMAP_X_AXIS_COLOR);
		coloring_it.value()->addOption("Y-Axis", UFOMAP_Y_AXIS_COLOR);
		coloring_it.value()->addOption("Z-Axis", UFOMAP_Z_AXIS_COLOR);
		coloring_it.value()->addOption("Cell Probability", UFOMAP_PROBABLILTY_COLOR);
		coloring_it.value()->addOption("Fixed", UFOMAP_FIXED_COLOR);

		auto color_factor_it = color_factor_property_.insert(
				type, new rviz::FloatProperty("Factor", 0.8, "", coloring_it.value(),
																			SLOT(updateOctreeColorMode()), this));
		color_factor_it.value()->setMin(0.0);
		color_factor_it.value()->setMax(1.0);
		if ("Voxel Color" == default_coloring || "Cell Probability" == default_coloring ||
				"Fixed" == default_coloring)
		{
			color_factor_it.value()->hide();
		}

		auto color_it = color_property_.insert(
				type, new rviz::ColorProperty("Color", default_color, "", coloring_it.value(),
																			SLOT(updateOctreeColorMode()), this));
		if ("Voxel Color" == default_coloring || "X-Axis" == default_coloring ||
				"Y-Axis" == default_coloring || "Z-Axis" == default_coloring)
		{
			color_it.value()->hide();
		}

		auto alpha_it = alpha_property_.insert(
				type,
				new rviz::FloatProperty("Alpha", default_alpha, "Set voxel transparency alpha",
																it.value(), SLOT(updateAlpha()), this));
		alpha_it.value()->setMin(0.0);
		alpha_it.value()->setMax(1.0);
	}

	depth_property_ = new rviz::IntProperty("Min. Depth", 0, "", this, SLOT(updateDepth()));
	depth_property_->setMin(0);
	depth_property_->setMax(MAX_OCTREE_DEPTH_LEVELS_);

	use_bbx_property_ =
			new rviz::BoolProperty("Use BBX", false, "", this, SLOT(updateBBX()), this);
	use_bbx_property_->setDisableChildrenIfFalse(true);
	min_bbx_property_ = new rviz::VectorProperty(
			"Minimum",
			Ogre::Vector3(-1000),  // TODO: lowest?
			"Defines the minimum BBX to display", use_bbx_property_, SLOT(updateBBX()), this);

	max_bbx_property_ = new rviz::VectorProperty(
			"Maximum", Ogre::Vector3(1000), "Defines the maximum BBX to display",
			use_bbx_property_, SLOT(updateBBX()), this);
}

OctreeRGBDisplay::~OctreeRGBDisplay()
{
	// TODO: Implement
}

void OctreeRGBDisplay::update(float wall_dt, float ros_dt)
{
	if (should_update_)
	{
		const std::lock_guard<std::mutex> lock(mutex_);

		QHash<OctreeVoxelType, std::vector<std::vector<rviz::PointCloud::Point>>> points;
		QHash<OctreeVoxelType, std::vector<std::vector<float>>> probabilities;
		for (const OctreeVoxelType& type : { UFOMAP_OCCUPIED, UFOMAP_FREE, UFOMAP_UNKNOWN })
		{
			points[type].resize(clouds_[type].size());
			probabilities[type].resize(clouds_[type].size());
		}

		if (render_type_[UFOMAP_OCCUPIED]->getBool() ||
				render_type_[UFOMAP_FREE]->getBool() || render_type_[UFOMAP_UNKNOWN]->getBool())
		{
			ufomap::Point3 min_value = ufomap_.getMin();
			ufomap::Point3 max_value = ufomap_.getMax();

			if (use_bbx_property_->getBool())
			{
				Ogre::Vector3 min_bbx = min_bbx_property_->getVector();
				Ogre::Vector3 max_bbx = max_bbx_property_->getVector();

				for (int i = 0; i < 3; ++i)
				{
					min_value[i] = std::max(min_value[i], min_bbx[i]);
					max_value[i] = std::min(max_value[i], max_bbx[i]);
				}
			}

			QHash<OctreeVoxelType, ufomap::Point3> min_coord;
			QHash<OctreeVoxelType, ufomap::Point3> max_coord;

			for (auto it = ufomap_.begin_leafs_bounding(
										ufomap_geometry::AABB(min_value, max_value),
										render_type_[UFOMAP_OCCUPIED]->getBool(),
										render_type_[UFOMAP_FREE]->getBool(),
										render_type_[UFOMAP_UNKNOWN]->getBool(), false,
										depth_property_->getInt()),
								end = ufomap_.end_leafs();
					 it != end; ++it)
			{
				// TODO: Check visibility - no, bad idea!

				ufomap::Point3 coord = it.getCenter();
				rviz::PointCloud::Point point;
				point.position.x = coord.x();
				point.position.y = coord.y();
				point.position.z = coord.z();

				OctreeVoxelType type;
				if (it.isOccupied())
				{
					type = UFOMAP_OCCUPIED;
				}
				else if (it.isFree())
				{
					type = UFOMAP_FREE;
				}
				else
				{
					type = UFOMAP_UNKNOWN;
				}

				if (min_coord.contains(type))
				{
					for (int i = 0; i < 3; ++i)
					{
						min_coord[type][i] = std::min(min_coord[type][i], coord[i]);
						max_coord[type][i] = std::max(max_coord[type][i], coord[i]);
					}
				}
				else
				{
					min_coord[type] = coord;
					max_coord[type] = coord;
				}

				if (UFOMAP_OCCUPIED == type &&
						UFOMAP_VOXEL_COLOR == coloring_property_[UFOMAP_OCCUPIED]->getOptionInt())
				{
					point.setColor(it->node->color.r / 255.0, it->node->color.g / 255.0,
												 it->node->color.b / 255.0, it.getProbability());
				}
				else
				{
					probabilities[type][it.getDepth()].push_back(it.getProbability());
				}

				points[type][it.getDepth()].push_back(point);
			}

			for (const OctreeVoxelType& type : points.keys())
			{
				if (UFOMAP_OCCUPIED != type ||
						UFOMAP_VOXEL_COLOR != coloring_property_[type]->getOptionInt())
				{
					// Color points
					for (size_t i = 0; i < points[type].size(); ++i)
					{
						for (size_t j = 0; j < points[type][i].size(); ++j)
						{
							colorPoint(points[type][i][j], min_coord[type], max_coord[type],
												 probabilities[type][i][j], type);
						}
					}
				}
			}
		}

		for (const OctreeVoxelType& type : clouds_.keys())
		{
			for (size_t i = 0; i < clouds_[type].size(); ++i)
			{
				clouds_[type][i].clear();
				clouds_[type][i].setAlpha(alpha_property_[type]->getFloat());
				if (i < ufomap_.getTreeDepthLevels())
				{
					clouds_[type][i].setDimensions(ufomap_.getNodeSize(i), ufomap_.getNodeSize(i),
																				 ufomap_.getNodeSize(i));
				}
				if (!points[type][i].empty())
				{
					clouds_[type][i].addPoints(&points[type][i].front(), points[type][i].size());
				}
			}
		}

		updateOctreeRenderStyle();  // TODO: Should not have to call this here

		should_update_ = false;
	}
	updateFromTF();
}

void OctreeRGBDisplay::octreeCallback(const ufomap_msgs::Ufomap::ConstPtr& msg)
{
	++num_messages_received_;
	setStatus(rviz::StatusProperty::Ok, "Messages",
						QString::number(num_messages_received_) + " ufomap messages received");
	setStatusStd(rviz::StatusProperty::Ok, "Type", msg->info.id.c_str());
	if (!checkType(msg->info.id))
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

	if (!ufomap_msgs::msgToUfomap(*msg, ufomap_))
	{
		setStatusStd(rviz::StatusProperty::Error, "Message", "Could not create octree.");
	}

	depth_property_->setMax(ufomap_.getTreeDepthLevels());

	should_update_ = true;

	// TODO: Do something with tree depth
}

void OctreeRGBDisplay::colorPoint(rviz::PointCloud::Point& point,
																	const ufomap::Point3& min_value,
																	const ufomap::Point3& max_value, float probability,
																	OctreeVoxelType type) const
{
	switch (coloring_property_[type]->getOptionInt())
	{
		case UFOMAP_X_AXIS_COLOR:
			setColor(point.position.x, min_value.x(), max_value.x(),
							 color_factor_property_[type]->getFloat(), point);
			break;
		case UFOMAP_Y_AXIS_COLOR:
			setColor(point.position.y, min_value.y(), max_value.y(),
							 color_factor_property_[type]->getFloat(), point);
			break;
		case UFOMAP_Z_AXIS_COLOR:
			setColor(point.position.z, min_value.z(), max_value.z(),
							 color_factor_property_[type]->getFloat(), point);
			break;
		case UFOMAP_PROBABLILTY_COLOR:
		{
			QColor color = color_property_[type]->getColor();
			float prob = probability;
			if (UFOMAP_FREE == type)
			{
				prob = 1.0 - prob;
			}
			point.setColor(prob * (color.red() / 255.0), prob * (color.green() / 255.0),
										 prob * (color.blue() / 255.0));
			break;
		}
		case UFOMAP_FIXED_COLOR:
		{
			QColor color = color_property_[type]->getColor();
			point.setColor(color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0);
			break;
		}
	}
}

void OctreeRGBDisplay::updateOctreeColorMode()
{
	for (const auto& type : coloring_property_.keys())
	{
		switch (coloring_property_[type]->getOptionInt())
		{
			case UFOMAP_VOXEL_COLOR:
				color_factor_property_[type]->hide();
				color_property_[type]->hide();
				break;
			case UFOMAP_PROBABLILTY_COLOR:
			case UFOMAP_FIXED_COLOR:
				color_factor_property_[type]->hide();
				color_property_[type]->show();
				break;
			default:
				color_factor_property_[type]->show();
				color_property_[type]->hide();
				break;
		}
	}

	should_update_ = true;
}

bool OctreeRGBDisplay::checkType(const std::string& type) const
{
	return type == ufomap_.getTreeType();
}

void OctreeRGBDisplay::setTopic(const QString& topic, const QString& datatype)
{
	topic_property_->setString(topic);
}
}  // namespace ufomap_rviz_plugin

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ufomap_rviz_plugin::OctreeRGBDisplay, rviz::Display)