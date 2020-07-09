#ifndef UFOMAP_ITERATOR_BASE_H
#define UFOMAP_ITERATOR_BASE_H

#include <ufomap/node.h>
#include <ufomap/types.h>

#include <stack>
#include <vector>

namespace ufomap
{
template <typename TREE, typename INNER_NODE, typename LEAF_NODE>
class BaseIterator
{
public:
	bool operator==(const BaseIterator& rhs) const
	{
		return (rhs.tree_ == tree_ && rhs.stack_.size() == stack_.size() &&
						(stack_.empty() || (rhs.stack_.top() == stack_.top())));
	}

	bool operator!=(const BaseIterator& rhs) const
	{
		return !(*this == rhs);
		// (rhs.tree_ != tree_ || rhs.stack_.size() != stack_.size() ||
		// 				(!stack_.empty() && rhs.stack_.top() != stack_.top()));
	}

	const Node<LEAF_NODE>* operator->() const
	{
		return &(stack_.top());  // TODO: What should this be?
	}

	Node<LEAF_NODE>& operator*() const
	{
		return stack_.top();
	}

	bool isOccupied() const
	{
		return tree_->isOccupied(stack_.top());
	}

	bool isFree() const
	{
		return tree_->isFree(stack_.top());
	}

	bool isUnknown() const
	{
		return tree_->isUnknown(stack_.top());
	}

	bool containsOccupied() const
	{
		if (isPureLeaf())
		{
			return isOccupied();
		}
		return tree_->containsOccupied(stack_.top());
	}

	bool containsFree() const
	{
		if (isPureLeaf())
		{
			return isFree();
		}
		return tree_->containsFree(stack_.top());
	}

	bool containsUnknown() const
	{
		if (isPureLeaf())
		{
			return isUnknown();
		}
		return tree_->containsUnknown(stack_.top());
	}

	double getProbability() const
	{
		return tree_->probability(stack_.top());
	}

	double getLogit() const
	{
		return tree_->logit(stack_.top());
	}

	double getSize() const
	{
		return tree_->getNodeSize(stack_.top().getDepth());
	}

	double getHalfSize() const
	{
		return tree_->getNodeHalfSize(stack_.top().getDepth());
	}

	unsigned int getDepth() const
	{
		return stack_.top().getDepth();
	}

	Point3 getCenter() const
	{
		return tree_->keyToCoord(stack_.top().code.toKey());
	}

	double getX() const
	{
		return tree_->keyToCoord(stack_.top().code.toKey(0), getDepth());
	}
	double getY() const
	{
		return tree_->keyToCoord(stack_.top().code.toKey(1), getDepth());
	}
	double getZ() const
	{
		return tree_->keyToCoord(stack_.top().code.toKey(2), getDepth());
	}

	bool isPureLeaf() const
	{
		return 0 == getDepth();
	}

	bool isLeaf() const
	{
		return isPureLeaf() || tree_->isLeaf(stack_.top());
	}

public:
	// iterator traits
	using difference_type = std::ptrdiff_t;  // What should this be?
	using value_type = Node<LEAF_NODE>;
	using pointer = const Node<LEAF_NODE>*;  // Should be const?
	using reference = Node<LEAF_NODE>&;      // Should be const?
	using iterator_category = std::forward_iterator_tag;

protected:
	BaseIterator(const TREE* tree = nullptr, bool occupied_space = true,
							 bool free_space = true, bool unknown_space = true, bool contains = false,
							 unsigned int min_depth = 0)
		: tree_(tree)
		, min_depth_(min_depth)
		, occupied_space_(occupied_space)
		, free_space_(free_space)
		, unknown_space_(unknown_space)
		, contains_(contains)
	{
	}

	BaseIterator(const BaseIterator& other)
		: tree_(other.tree_)
		, min_depth_(other.min_depth_)
		, stack_(other.stack_)
		, occupied_space_(other.occupied_space_)
		, free_space_(other.free_space_)
		, unknown_space_(other.unknown_space_)
		, contains_(other.contains_)
	{
	}

	BaseIterator& operator=(const BaseIterator& rhs)
	{
		tree_ = rhs.tree_;
		min_depth_ = rhs.min_depth_;
		stack_ = rhs.stack_;
		occupied_space_ = rhs.occupied_space_;
		free_space_ = rhs.free_space_;
		unknown_space_ = rhs.unknown_space_;
		contains_ = rhs.contains_;
		return *this;
	}

	virtual bool validNode(const Node<LEAF_NODE>& node) const
	{
		if (contains_ || min_depth_ != node.getDepth())
		{
			return (occupied_space_ && tree_->containsOccupied(node)) ||
						 (free_space_ && tree_->containsFree(node)) ||
						 (unknown_space_ && tree_->containsUnknown(node));
		}

		return (occupied_space_ && tree_->isOccupied(node)) ||
					 (free_space_ && tree_->isFree(node)) ||
					 (unknown_space_ && tree_->isUnknown(node));
	}

	virtual bool validReturnNode(const Node<LEAF_NODE>& node) const
	{
		if (contains_)
		{
			return (occupied_space_ && tree_->containsOccupied(node)) ||
						 (free_space_ && tree_->containsFree(node)) ||
						 (unknown_space_ && tree_->containsUnknown(node));
		}

		return (occupied_space_ && tree_->isOccupied(node)) ||
					 (free_space_ && tree_->isFree(node)) ||
					 (unknown_space_ && tree_->isUnknown(node));
	}

	virtual void increment()
	{
		if (this->stack_.empty())
		{
			this->tree_ = nullptr;
		}
		else
		{
			if (!this->stack_.empty())
			{
				// Skip forward to next valid node
				do
				{
					this->singleIncrement();
				} while (!this->stack_.empty() && !validReturnNode(this->stack_.top()));
			}

			if (this->stack_.empty())
			{
				this->tree_ = nullptr;
			}
		}
	}

	virtual void singleIncrement()
	{
		if (this->getDepth() <= this->min_depth_ || this->isLeaf())
		{
			this->stack_.pop();
			return;
		}

		Node<LEAF_NODE> top = stack_.top();
		stack_.pop();

		// We know it is a inner node
		const INNER_NODE* inner_node = static_cast<const INNER_NODE*>(top.node);

		Node<LEAF_NODE> node;
		unsigned int child_depth = top.getDepth() - 1;
		// Do it in reverse so they come in order in the stack
		for (int i = 7; 0 <= i; --i)
		{
			node.code = top.code.getChild(i);

			if (0 == child_depth)
			{
				node.node = &((*static_cast<std::array<LEAF_NODE, 8>*>(inner_node->children))[i]);
			}
			else
			{
				node.node =
						&((*static_cast<std::array<INNER_NODE, 8>*>(inner_node->children))[i]);
			}

			if (validNode(node))
			{
				stack_.push(node);
			}
		}
	}

protected:
	const TREE* tree_;
	unsigned int min_depth_;

	std::stack<Node<LEAF_NODE>, std::vector<Node<LEAF_NODE>>> stack_;

	bool occupied_space_;
	bool free_space_;
	bool unknown_space_;

	bool contains_;
};
}  // namespace ufomap

#endif  // UFOMAP_ITERATOR_BASE_H
