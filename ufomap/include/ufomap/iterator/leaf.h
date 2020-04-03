#ifndef UFOMAP_ITERATOR_LEAF_H
#define UFOMAP_ITERATOR_LEAF_H

#include <ufomap/iterator/tree.h>

namespace ufomap
{
template <typename TREE, typename INNER_NODE, typename LEAF_NODE>
class LeafIterator : public TreeIterator<TREE, INNER_NODE, LEAF_NODE>
{
public:
	LeafIterator() : TreeIterator<TREE, INNER_NODE, LEAF_NODE>()
	{
	}

	LeafIterator(const TREE* tree, const ufomap_geometry::BoundingVolume& bounding_volume,
							 bool occupied_space = true, bool free_space = true,
							 bool unknown_space = true, bool contains = false,
							 unsigned int min_depth = 0)
		: TreeIterator<TREE, INNER_NODE, LEAF_NODE>(tree, bounding_volume, occupied_space,
																								free_space, unknown_space, contains,
																								min_depth)
	{
		if (!this->stack_.empty())
		{
			if (!validReturnNode(this->stack_.top()))
			{
				operator++();
			}
		}
	}

	LeafIterator(const LeafIterator& other)
		: TreeIterator<TREE, INNER_NODE, LEAF_NODE>(other)
	{
	}

	LeafIterator& operator=(const LeafIterator& rhs)
	{
		TreeIterator<TREE, INNER_NODE, LEAF_NODE>::operator=(rhs);
		return *this;
	}

	// Postfix increment
	LeafIterator operator++(int)
	{
		LeafIterator result = *this;
		++(*this);
		return result;
	}

	// Prefix increment
	LeafIterator& operator++()
	{
		this->increment();
		return *this;
	}

protected:
	virtual bool validReturnNode(const Node<LEAF_NODE>& node) const override
	{
		return (this->tree_->isLeaf(node) || node.getDepth() == this->min_depth_) &&
					 TreeIterator<TREE, INNER_NODE, LEAF_NODE>::validReturnNode(node);
	}
};

}  // namespace ufomap

#endif  // UFOMAP_ITERATOR_LEAF_H
