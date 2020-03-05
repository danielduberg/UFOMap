#ifndef UFOMAP_ITERATOR_TREE_H
#define UFOMAP_ITERATOR_TREE_H

#include <ufomap/geometry/intersects.h>
#include <ufomap/iterator/base.h>

#include <type_traits>
namespace ufomap
{
template <typename, typename = void>
struct hasBegin : std::false_type
{
};

template <typename T>
struct hasBegin<T, std::void_t<decltype(((T*)nullptr)->begin())>> : std::true_type
{
};

template <typename, typename = void>
struct hasEnd : std::false_type
{
};

template <typename T>
struct hasEnd<T, std::void_t<decltype(((T*)nullptr)->end())>> : std::true_type
{
};

template <typename TREE, typename INNER_NODE, typename LEAF_NODE, typename BOUNDING_TYPE>
class TreeIterator : public BaseIterator<TREE, INNER_NODE, LEAF_NODE>
{
public:
	TreeIterator() : BaseIterator<TREE, INNER_NODE, LEAF_NODE>()
	{
	}

	TreeIterator(const TREE* tree, const BOUNDING_TYPE& bounding_type,
							 bool occupied_space = true, bool free_space = true,
							 bool unknown_space = true, bool contains = false,
							 unsigned int min_depth = 0)
		: BaseIterator<TREE, INNER_NODE, LEAF_NODE>(tree, occupied_space, free_space,
																								unknown_space, contains, min_depth)
		, bounding_type_(bounding_type)
	{
		// if constexpr (std::is_same<BOUNDING_TYPE, ufomap_geometry::AABB>::value)
		// {
		// 	Point3 min = bounding_type.getMin();
		// 	Point3 max = bounding_type.getMax();

		// 	// if (tree->isBBXLimitEnabled())
		// 	// {
		// 	// 	Point3 tree_bbx_min = tree->getBBXMin();
		// 	// 	Point3 tree_bbx_max = tree->getBBXMax();

		// 	// 	for (int i = 0; i < 3; ++i)
		// 	// 	{
		// 	// 		min[i] = std::max(min[i], tree_bbx_min[i]);
		// 	// 		max[i] = std::min(max[i], tree_bbx_max[i]);
		// 	// 	}
		// 	// }

		// 	Key min_key, max_key;
		// 	if (!tree->coordToKeyChecked(min, min_key, min_depth) ||
		// 			!tree->coordToKeyChecked(max, max_key, min_depth))
		// 	{
		// 		// Same as end iterator
		// 		this->tree_ = nullptr;
		// 		this->min_depth_ = 0;
		// 		return;
		// 	}
		// 	else
		// 	{
		// 		bbx_min_ = Code(min_key);
		// 		bbx_max_ = Code(max_key);
		// 	}
		// }

		Node<LEAF_NODE> root = this->tree_->getRoot();
		if (nullptr != this->tree_ && (occupied_space || free_space || unknown_space) &&
				validNode(root) && root.getDepth() >= min_depth)  // FIXME: Is last correct?
		{
			this->stack_.push(root);
			if (!this->validReturnNode(root))
			{
				operator++();
			}
		}
		else
		{
			// Same as end iterator
			this->tree_ = nullptr;
			this->min_depth_ = 0;
		}
	}

	TreeIterator(const TreeIterator& other)
		: BaseIterator<TREE, INNER_NODE, LEAF_NODE>(other)
		// , bbx_min_(other.bbx_min_)
		// , bbx_max_(other.bbx_max_)
		, bounding_type_(other.bounding_type_)
	{
	}

	TreeIterator& operator=(const TreeIterator& rhs)
	{
		BaseIterator<TREE, INNER_NODE, LEAF_NODE>::operator=(rhs);
		// bbx_min_ = rhs.bbx_min_;
		// bbx_max_ = rhs.bbx_max_;
		bounding_type_ = rhs.bounding_type_;
		return *this;
	}

	// Postfix increment
	TreeIterator operator++(int)
	{
		TreeIterator result = *this;
		++(*this);
		return result;
	}

	// Prefix increment
	TreeIterator& operator++()
	{
		this->increment();
		return *this;
	}

protected:
	virtual bool validNode(const Node<LEAF_NODE>& node) const override
	{
		// if constexpr (std::is_same<BOUNDING_TYPE, ufomap_geometry::AABB>::value)
		// {
		// 	return BaseIterator<TREE, INNER_NODE, LEAF_NODE>::validNode(node) &&
		// 				 (bbx_min_.toDepth(node.getDepth()) <= node.code && bbx_max_ >= node.code);
		// }

		if (!BaseIterator<TREE, INNER_NODE, LEAF_NODE>::validNode(node))
		{
			return false;
		}

		Point3 center = this->tree_->keyToCoord(node.code.toKey());
		float half_size = this->tree_->getNodeHalfSize(node.getDepth());
		ufomap_geometry::AABB aabb(center - half_size, center + half_size);

		if constexpr (hasBegin<BOUNDING_TYPE>::value && hasEnd<BOUNDING_TYPE>::value)
		{
			for (auto it = bounding_type_.begin(), it_end = bounding_type_.end(); it != it_end;
					 ++it)
			{
				if (ufomap_geometry::intersects(aabb, *it))
				{
					return true;
				}
			}
			return false;
		}
		else
		{
			return ufomap_geometry::intersects(aabb, bounding_type_);
		}
	}

protected:
	// Code bbx_min_;
	// Code bbx_max_;
	BOUNDING_TYPE bounding_type_;
};

}  // namespace ufomap

#endif  // UFOMAP_ITERATOR_TREE_H
