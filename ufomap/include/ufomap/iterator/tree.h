#ifndef UFOMAP_ITERATOR_TREE_H
#define UFOMAP_ITERATOR_TREE_H

#include <ufomap/geometry/intersects.h>
#include <ufomap/geometry/types.h>
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

	TreeIterator(const TREE* tree, const BOUNDING_TYPE& bounding_volume,
							 bool occupied_space = true, bool free_space = true,
							 bool unknown_space = true, bool contains = false,
							 unsigned int min_depth = 0)
		: BaseIterator<TREE, INNER_NODE, LEAF_NODE>(tree, occupied_space, free_space,
																								unknown_space, contains, min_depth)
		, bounding_volume_(bounding_volume)
	{
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
		, bounding_volume_(other.bounding_volume_)
	{
	}

	TreeIterator& operator=(const TreeIterator& rhs)
	{
		BaseIterator<TREE, INNER_NODE, LEAF_NODE>::operator=(rhs);
		bounding_volume_ = rhs.bounding_volume_;
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
		if (!BaseIterator<TREE, INNER_NODE, LEAF_NODE>::validNode(node))
		{
			return false;
		}

		Point3 center = this->tree_->keyToCoord(node.code.toKey());
		float half_size = this->tree_->getNodeHalfSize(node.getDepth());
		ufomap_geometry::AABB aabb(center - half_size, center + half_size);

		if constexpr (hasBegin<BOUNDING_TYPE>::value && hasEnd<BOUNDING_TYPE>::value)
		{
			for (auto& v : bounding_volume_)
			{
				if constexpr (std::is_same_v<BoundingVar, v>)
				{
					if (std::visit([aabb](auto&& arg)
														 -> bool { return ufomap_geometry::intersects(aabb, arg); },
												 v))
					{
						return true;
					}
				}
				else
				{
					if (ufomap_geometry::intersects(aabb, v))
					{
						return true;
					}
				}
			}
			return false;
		}
		else if constexpr (std::is_same_v<BoundingVar, bounding_volume_>)
		{
			return std::visit(
					[aabb](auto&& arg) -> bool { return ufomap_geometry::intersects(aabb, arg); },
					bounding_volume_);
		}
		else
		{
			return ufomap_geometry::intersects(aabb, bounding_volume_);
		}
	}

protected:
	BOUNDING_TYPE bounding_volume_;
};

}  // namespace ufomap

#endif  // UFOMAP_ITERATOR_TREE_H
