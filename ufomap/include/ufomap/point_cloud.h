#ifndef UFOMAP_POINT_CLOUD_H
#define UFOMAP_POINT_CLOUD_H

#include <ufomap/math/pose6.h>
#include <ufomap/types.h>

#include <type_traits>
#include <vector>

namespace ufomap
{
/**
 * @brief A collection of 3D coordinates of type T
 *
 * @tparam T The type of points to store in the point cloud. Has to inhert from Point3
 */
template <typename T, typename = std::enable_if_t<std::is_base_of_v<Point3, T>>>
class PointCloudT
{
public:
	PointCloudT()
	{
	}

	PointCloudT(const PointCloudT& other)
	{
		cloud_.reserve(other.size());
		push_back(other);
	}

	~PointCloudT()
	{
	}

	/**
	 * @brief Access specified point
	 *
	 * @param index Index of the point to return
	 * @return const T& Reference to the requested point
	 */
	inline const T& operator[](size_t index) const
	{
		return cloud_[index];
	}

	/**
	 * @brief Access specified point
	 *
	 * @param index Index of the point to return
	 * @return const T& Reference to the requested point
	 */
	inline T& operator[](size_t index)
	{
		return cloud_[index];
	}

	/**
	 * @brief Erases all points from the point cloud
	 *
	 */
	inline void clear()
	{
		cloud_.clear();
	}

	/**
	 * @brief Increase the capacity of the point cloud to a value that is greater or equal
	 * to new_cap
	 *
	 * @param new_cap New capacity of the point cloud
	 */
	inline void reserve(size_t new_cap)
	{
		cloud_.reserve(new_cap);
	}

	/**
	 * @brief Return the number of points in the point cloud
	 *
	 * @return size_t The number of points in the point cloud
	 */
	inline size_t size() const
	{
		return cloud_.size();
	}

	/**
	 * @brief Adds a point to the point cloud
	 *
	 * @param point The point to add to the point cloud
	 */
	inline void push_back(const T& point)
	{
		cloud_.push_back(point);
	}

	/**
	 * @brief Adds all points from another point cloud to this point cloud
	 *
	 * @param other The other point cloud that points should be added from
	 */
	inline void push_back(const PointCloudT& other)
	{
		cloud_.insert(cloud_.end(), other.begin(), other.end());
	}

	/**
	 * @brief Transform each point in the point cloud
	 *
	 * @param transform The transformation to be applied to each point
	 */
	inline void transform(const ufomap_math::Pose6& transform)
	{
		for (Point3& point : cloud_)
		{
			point = transform.transform(point);
		}
	}

	/**
	 * @brief Returns an iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::iterator Iterator to the first point
	 */
	inline typename std::vector<T>::iterator begin()
	{
		return cloud_.begin();
	}

	/**
	 * @brief Returns an iterator to the last element following the last point of the point
	 * cloud
	 *
	 * @return std::vector<T>::iterator Iterator to the element following the last point
	 */
	inline typename std::vector<T>::iterator end()
	{
		return cloud_.end();
	}

	/**
	 * @brief Returns an iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::const_iterator Iterator to the first point
	 */
	inline typename std::vector<T>::const_iterator begin() const
	{
		return cloud_.begin();
	}

	/**
	 * @brief Returns an iterator to the last element following the last point of the point
	 * cloud
	 *
	 * @return std::vector<T>::const_iterator Iterator to the element following the last
	 * point
	 */
	inline typename std::vector<T>::const_iterator end() const
	{
		return cloud_.end();
	}

	/**
	 * @brief Returns an iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::const_iterator Iterator to the first point
	 */
	inline typename std::vector<T>::const_iterator cbegin() const
	{
		return cloud_.cbegin();
	}

	/**
	 * @brief Returns an iterator to the last element following the last point of the point
	 * cloud
	 *
	 * @return std::vector<T>::const_iterator Iterator to the element following the last
	 * point
	 */
	inline typename std::vector<T>::const_iterator cend() const
	{
		return cloud_.cend();
	}

	/**
	 * @brief Returns a reverse iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::reverse_iterator Reverse iterator to the first point
	 */
	inline typename std::vector<T>::reverse_iterator rbegin()
	{
		return cloud_.rbegin();
	}

	/**
	 * @brief Returns a reverse iterator to the element following the last point of the
	 * reversed point cloud
	 *
	 * @return std::vector<T>::reverse_iterator Reverse iterator to the element following
	 * the last point
	 */
	inline typename std::vector<T>::reverse_iterator rend()
	{
		return cloud_.rend();
	}

	/**
	 * @brief Returns a reverse iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::const_reverse_iterator Reverse iterator to the first point
	 */
	inline typename std::vector<T>::const_reverse_iterator rbegin() const
	{
		return cloud_.rbegin();
	}

	/**
	 * @brief Returns a reverse iterator to the element following the last point of the
	 * reversed point cloud
	 *
	 * @return std::vector<T>::const_reverse_iterator Reverse iterator to the element
	 * following the last point
	 */
	inline typename std::vector<T>::const_reverse_iterator rend() const
	{
		return cloud_.rend();
	}

	/**
	 * @brief Returns a reverse iterator to the first point of the point cloud
	 *
	 * @return std::vector<T>::const_reverse_iterator Reverse iterator to the first point
	 */
	inline typename std::vector<T>::const_reverse_iterator crbegin() const
	{
		return cloud_.rbegin();
	}

	/**
	 * @brief Returns a reverse iterator to the element following the last point of the
	 * reversed point cloud
	 *
	 * @return std::vector<T>::const_reverse_iterator Reverse iterator to the element
	 * following the last point
	 */
	inline typename std::vector<T>::const_reverse_iterator crend() const
	{
		return cloud_.rend();
	}

private:
	std::vector<T> cloud_;  // The point cloud
};

using PointCloud = PointCloudT<Point3>;
using PointCloudRGB = PointCloudT<Point3RGB>;
using PointCloudI = PointCloudT<Point3I>;

}  // namespace ufomap

#endif  // UFOMAP_POINT_CLOUD_H