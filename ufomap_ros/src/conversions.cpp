#include <ufomap/types.h>
#include <ufomap_ros/conversions.h>

#include <sensor_msgs/point_cloud2_iterator.h>

// TODO: Add support for intensity?

namespace ufomap
{
void getFields(sensor_msgs::PointCloud2::ConstPtr cloud, bool& has_x, bool& has_y,
							 bool& has_z, bool& has_rgb, bool& has_i)
{
	has_x = false;
	has_y = false;
	has_z = false;
	has_rgb = false;
	has_i = false;

	for (const auto& field : cloud->fields)
	{
		if ("x" == field.name)
		{
			has_x = true;
		}
		else if ("y" == field.name)
		{
			has_y = true;
		}
		else if ("z" == field.name)
		{
			has_z = true;
		}
		else if ("rgb" == field.name)
		{
			has_rgb = true;
		}
		else if ("r" == field.name)
		{
			has_rgb = true;
		}
		else if ("g" == field.name)
		{
			has_rgb = true;
		}
		else if ("b" == field.name)
		{
			has_rgb = true;
		}
		else if ("intensity" == field.name)
		{
			has_i = true;
		}
	}
}

void toUfomap(sensor_msgs::PointCloud2::ConstPtr cloud_in, PointCloud& cloud_out)
{
	cloud_out.reserve(cloud_in->data.size() / cloud_in->point_step);

	bool has_x, has_y, has_z, has_rgb, has_i;
	getFields(cloud_in, has_x, has_y, has_z, has_rgb, has_i);

	if (!has_x || !has_y || !has_z)
	{
		throw std::runtime_error("cloud_in missing one or more of the xyz fields");
	}

	sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_in, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_in, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_in, "z");

	for (int i = 0; iter_x != iter_x.end(); ++i, ++iter_x, ++iter_y, ++iter_z)
	{
		if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z))
		{
			cloud_out.push_back(Point3(*iter_x, *iter_y, *iter_z));
		}
	}
}

void fromUfomap(const PointCloud& cloud_in, sensor_msgs::PointCloud2::Ptr cloud_out)
{
	bool has_x, has_y, has_z, has_rgb, has_i;
	getFields(cloud_out, has_x, has_y, has_z, has_rgb, has_i);

	// if (!has_x || !has_y || !has_z)
	// {
	// 	throw std::runtime_error("cloud_out missing one or more of the xyz fields");
	// }

	// TODO: Is these two needed?
	sensor_msgs::PointCloud2Modifier cloud_out_modifier(*cloud_out);
	cloud_out_modifier.setPointCloud2FieldsByString(1, "xyz");
	cloud_out_modifier.resize(cloud_in.size());

	sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_out, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_out, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_out, "z");

	for (unsigned int i = 0; i < cloud_in.size(); ++i, ++iter_x, ++iter_y, ++iter_z)
	{
		*iter_x = cloud_in[i][0];
		*iter_y = cloud_in[i][1];
		*iter_z = cloud_in[i][2];
	}
}

void toUfomap(sensor_msgs::PointCloud2::ConstPtr cloud_in, PointCloudRGB& cloud_out)
{
	cloud_out.reserve(cloud_in->data.size() / cloud_in->point_step);

	bool has_x, has_y, has_z, has_rgb, has_i;
	getFields(cloud_in, has_x, has_y, has_z, has_rgb, has_i);

	if (!has_x || !has_y || !has_z)
	{
		throw std::runtime_error("cloud_in missing one or more of the xyz fields");
	}

	if (!has_rgb)
	{
		throw std::runtime_error("cloud_in missing one or more of the rgb fields");
	}

	sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_in, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_in, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_in, "z");
	sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_rgb(*cloud_in, "rgb");

	for (int i = 0; iter_x != iter_x.end(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
	{
		if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z) &&
				!std::isnan(iter_rgb[0]) && !std::isnan(iter_rgb[1]) && !std::isnan(iter_rgb[2]))
		{
			cloud_out.push_back(
					Point3RGB(*iter_x, *iter_y, *iter_z, iter_rgb[2], iter_rgb[1], iter_rgb[0]));
		}
	}
}

void fromUfomap(const PointCloudRGB& cloud_in, sensor_msgs::PointCloud2::Ptr cloud_out)
{
	bool has_x, has_y, has_z, has_rgb, has_i;
	getFields(cloud_out, has_x, has_y, has_z, has_rgb, has_i);

	// if (!has_x || !has_y || !has_z)
	// {
	// 	throw std::runtime_error("cloud_out missing one or more of the xyz fields");
	// }

	// if (!has_r || !has_g || !has_b)
	// {
	// 	throw std::runtime_error("cloud_out missing one or more of the rgb fields");
	// }

	// TODO: Is these two needed?
	sensor_msgs::PointCloud2Modifier cloud_out_modifier(*cloud_out);
	cloud_out_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
	cloud_out_modifier.resize(cloud_in.size());

	sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_out, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_out, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_out, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(*cloud_out, "rgb");

	for (unsigned int i = 0; i < cloud_in.size();
			 ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
	{
		*iter_x = cloud_in[i][0];
		*iter_y = cloud_in[i][1];
		*iter_z = cloud_in[i][2];
		iter_rgb[0] = cloud_in[i].getColor().r;
		iter_rgb[1] = cloud_in[i].getColor().g;
		iter_rgb[2] = cloud_in[i].getColor().b;
	}
}

void toUfomap(sensor_msgs::PointCloud2::ConstPtr cloud_in, PointCloudI& cloud_out)
{
	cloud_out.reserve(cloud_in->data.size() / cloud_in->point_step);

	bool has_x, has_y, has_z, has_rgb, has_i;
	getFields(cloud_in, has_x, has_y, has_z, has_rgb, has_i);

	if (!has_x || !has_y || !has_z)
	{
		throw std::runtime_error("cloud_in missing one or more of the xyz fields");
	}

	if (!has_i)
	{
		throw std::runtime_error("cloud_in missing the intensity field");
	}

	sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_in, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_in, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_in, "z");
	sensor_msgs::PointCloud2ConstIterator<float> iter_i(*cloud_in, "intensity");

	for (int i = 0; iter_x != iter_x.end(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i)
	{
		if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z))
		{
			cloud_out.push_back(Point3I(*iter_x, *iter_y, *iter_z, *iter_i));
		}
	}
}

void fromUfomap(const PointCloudI& cloud_in, sensor_msgs::PointCloud2::Ptr cloud_out)
{
	bool has_x, has_y, has_z, has_rgb, has_i;
	getFields(cloud_out, has_x, has_y, has_z, has_rgb, has_i);

	// if (!has_x || !has_y || !has_z)
	// {
	// 	throw std::runtime_error("cloud_out missing one or more of the xyz fields");
	// }

	// if (!has_r || !has_g || !has_b)
	// {
	// 	throw std::runtime_error("cloud_out missing one or more of the rgb fields");
	// }

	// TODO: Is these two needed?
	sensor_msgs::PointCloud2Modifier cloud_out_modifier(*cloud_out);
	cloud_out_modifier.setPointCloud2FieldsByString(2, "xyz", "intensity");
	cloud_out_modifier.resize(cloud_in.size());

	sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_out, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_out, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_out, "z");
	sensor_msgs::PointCloud2Iterator<float> iter_i(*cloud_out, "intensity");

	for (unsigned int i = 0; i < cloud_in.size();
			 ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i)
	{
		*iter_x = cloud_in[i][0];
		*iter_y = cloud_in[i][1];
		*iter_z = cloud_in[i][2];
		*iter_i = cloud_in[i].getIntensity();
	}
}

// Vector3
void toUfomap(const geometry_msgs::Vector3& vector3_in, ufomap_math::Vector3& vector3_out)
{
	vector3_out.x() = vector3_in.x;
	vector3_out.y() = vector3_in.y;
	vector3_out.z() = vector3_in.z;
}

ufomap_math::Vector3 toUfomap(const geometry_msgs::Vector3& vector3)
{
	return ufomap_math::Vector3(vector3.x, vector3.y, vector3.z);
}

void fromUfomap(const ufomap_math::Vector3& vector3_in,
								geometry_msgs::Vector3& vector3_out)
{
	vector3_out.x = vector3_in.x();
	vector3_out.y = vector3_in.y();
	vector3_out.z = vector3_in.z();
}

geometry_msgs::Vector3 fromUfomap(const ufomap_math::Vector3& vector3)
{
	geometry_msgs::Vector3 vector3_out;
	fromUfomap(vector3, vector3_out);
	return vector3_out;
}

// Quaternion
void toUfomap(const geometry_msgs::Quaternion& quaternion_in,
							ufomap_math::Quaternion& quaternion_out)
{
	quaternion_out.x() = quaternion_in.x;
	quaternion_out.y() = quaternion_in.y;
	quaternion_out.z() = quaternion_in.z;
	quaternion_out.w() = quaternion_in.w;
}

ufomap_math::Quaternion toUfomap(const geometry_msgs::Quaternion& quaternion)
{
	return ufomap_math::Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

void fromUfomap(const ufomap_math::Quaternion& quaternion_in,
								geometry_msgs::Quaternion& quaternion_out)
{
	quaternion_out.x = quaternion_in.x();
	quaternion_out.y = quaternion_in.y();
	quaternion_out.z = quaternion_in.z();
	quaternion_out.w = quaternion_in.w();
}

geometry_msgs::Quaternion fromUfomap(const ufomap_math::Quaternion& quaternion)
{
	geometry_msgs::Quaternion quaternion_out;
	fromUfomap(quaternion, quaternion_out);
	return quaternion_out;
}

// Transforms
void toUfomap(const geometry_msgs::Transform& transform_in,
							ufomap_math::Pose6& transform_out)
{
	toUfomap(transform_in.translation, transform_out.translation());
	toUfomap(transform_in.rotation, transform_out.rotation());
}

ufomap_math::Pose6 toUfomap(const geometry_msgs::Transform& transform)
{
	return ufomap_math::Pose6(transform.translation.x, transform.translation.y,
														transform.translation.z, transform.rotation.w,
														transform.rotation.x, transform.rotation.y,
														transform.rotation.z);
}

void fromUfomap(const ufomap_math::Pose6& transform_in,
								geometry_msgs::Transform& transform_out)
{
	fromUfomap(transform_in.translation(), transform_out.translation);
	fromUfomap(transform_in.rotation(), transform_out.rotation);
}

geometry_msgs::Transform fromUfomap(const ufomap_math::Pose6& transform)
{
	geometry_msgs::Transform transform_out;
	fromUfomap(transform, transform_out);
	return transform_out;
}

}  // namespace ufomap