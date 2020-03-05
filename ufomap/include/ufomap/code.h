#ifndef UFOMAP_CODE_H
#define UFOMAP_CODE_H

#include <ufomap/key.h>

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <stdint.h>
#include <x86intrin.h>

namespace ufomap
{
/**
 * @brief A code is a single value for indexing a specific node in an octree at a specific
 * depth
 *
 * @details Morton codes are used in UFOMap to increase performance when accessing the
 * octree
 *
 */
class Code
{
public:
	Code() : code_(0), depth_(0)
	{
	}

	Code(uint64_t code, unsigned int depth = 0) : code_(code), depth_(depth)
	{
	}

	Code(const Key& key) : code_(toCode(key)), depth_(key.getDepth())
	{
	}

	Code(const Code& other) : code_(other.code_), depth_(other.depth_)
	{
	}

	Code& operator=(const Code& rhs)
	{
		code_ = rhs.code_;
		depth_ = rhs.depth_;
		return *this;
	}

	bool operator==(const Code& rhs) const
	{
		return code_ == rhs.code_ && depth_ == rhs.depth_;
	}
	bool operator!=(const Code& rhs) const
	{
		return code_ != rhs.code_ || depth_ != rhs.depth_;
	}
	bool operator<(const Code& rhs) const
	{
		return get3Bits(code_) < get3Bits(rhs.code_) &&
					 get3Bits(code_ >> 1) < get3Bits(rhs.code_ >> 1) &&
					 get3Bits(code_ >> 2) < get3Bits(rhs.code_ >> 2);
	}
	bool operator<=(const Code& rhs) const
	{
		return get3Bits(code_) <= get3Bits(rhs.code_) &&
					 get3Bits(code_ >> 1) <= get3Bits(rhs.code_ >> 1) &&
					 get3Bits(code_ >> 2) <= get3Bits(rhs.code_ >> 2);
	}
	bool operator>(const Code& rhs) const
	{
		return get3Bits(code_) > get3Bits(rhs.code_) &&
					 get3Bits(code_ >> 1) > get3Bits(rhs.code_ >> 1) &&
					 get3Bits(code_ >> 2) > get3Bits(rhs.code_ >> 2);
	}
	bool operator>=(const Code& rhs) const
	{
		return get3Bits(code_) >= get3Bits(rhs.code_) &&
					 get3Bits(code_ >> 1) >= get3Bits(rhs.code_ >> 1) &&
					 get3Bits(code_ >> 2) >= get3Bits(rhs.code_ >> 2);
	}

	/**
	 * @brief Return the code at a specified depth
	 *
	 * @param depth The depth of the code
	 * @return Code The code at the specified depth
	 */
	Code toDepth(unsigned int depth) const
	{
		unsigned int temp = 3 * depth;
		return Code((code_ >> temp) << temp, depth);
	}

	/**
	 * @brief Converts a key to a code
	 *
	 * @param key The key to convert
	 * @return uint64_t The code corresponding to the key
	 */
	static uint64_t toCode(const Key& key)
	{
		return splitBy3(key[0]) | (splitBy3(key[1]) << 1) | (splitBy3(key[2]) << 2);
	}

	/**
	 * @brief Get the key component from a code
	 *
	 * @param code The code to generate the key component from
	 * @param index The index of the key component
	 * @return unsigned int The key component value
	 */
	static unsigned int toKey(const Code& code, unsigned int index)
	{
		return get3Bits(code.code_ >> index);
	}

	/**
	 * @brief Get the key component from this code
	 *
	 * @param index The index of the key component
	 * @return unsigned int The key component value
	 */
	unsigned int toKey(unsigned int index) const
	{
		return toKey(*this, index);
	}

	/**
	 * @brief Get the corresponding key to code
	 *
	 * @param code The code the corresponding key should be returned
	 * @return Key The corresponding key to code
	 */
	static Key toKey(const Code& code)
	{
		return Key(toKey(code, 0), toKey(code, 1), toKey(code, 2), code.getDepth());
	}

	/**
	 * @brief Get the corresponding key to this code
	 *
	 * @return Key The corresponding key to this code
	 */
	Key toKey() const
	{
		return toKey(*this);
	}

	/**
	 * @brief Get the child index at a specific depth for this code
	 *
	 * @param depth The depth the child index is requested for
	 * @return unsigned int The child index at the specified depth
	 */
	unsigned int getChildIdx(unsigned int depth) const
	{
		return (code_ >> static_cast<uint64_t>(3 * depth)) & ((uint64_t)0x7);
	}

	/**
	 * @brief Get the code of a specific child to this code
	 *
	 * @param index The index of the child
	 * @return Code The child code
	 */
	Code getChild(unsigned int index) const
	{
		if (0 == depth_)
		{
			// TODO: Throw error?
			return *this;
		}

		unsigned int child_depth = depth_ - 1;
		return Code(
				code_ + (static_cast<uint64_t>(index) << static_cast<uint64_t>(3 * child_depth)),
				child_depth);
	}

	/**
	 * @brief Get the eight child codes that comes from this code
	 *
	 * @return std::vector<Code> The eight child codes
	 */
	std::vector<Code> getChildren() const
	{
		std::vector<Code> children;
		if (0 == depth_)
		{
			return children;
		}

		unsigned int child_depth = depth_ - 1;
		uint64_t offset = 3 * child_depth;
		for (uint64_t i = 0; i < 8; ++i)
		{
			children.emplace_back(code_ + (i << offset), child_depth);
		}
		return children;
	}

	/**
	 * @brief Get all children that this code can have from this code's depth to depth 0
	 *
	 * @return std::vector<Code> Collection of all possible child codes of this code
	 */
	std::vector<Code> getAllChildren() const
	{
		std::vector<Code> children;
		uint64_t max = 8 << (3 * depth_);
		for (uint64_t i = 0; i < max; ++i)
		{
			children.emplace_back(code_ + i, 0);
		}
		return children;
	}

	/**
	 * @brief Get the code
	 *
	 * @return uint64_t The code
	 */
	uint64_t getCode() const
	{
		return code_;
	}

	/**
	 * @brief Get the depth that this code is specified at
	 *
	 * @return unsigned int The depth this code is specified at
	 */
	unsigned int getDepth() const
	{
		return depth_;
	}

	/**
	 * @brief
	 *
	 */
	struct CodeHash
	{
		size_t operator()(const Code& code) const
		{
			return code.code_;
		}
	};

private:
	__attribute__((target("default"))) static uint64_t splitBy3(unsigned int a)
	{
		uint64_t code = static_cast<uint64_t>(a) & 0x1fffff;
		code = (code | code << 32) & 0x1f00000000ffff;
		code = (code | code << 16) & 0x1f0000ff0000ff;
		code = (code | code << 8) & 0x100f00f00f00f00f;
		code = (code | code << 4) & 0x10c30c30c30c30c3;
		code = (code | code << 2) & 0x1249249249249249;
		return code;
	}

	__attribute__((target("bmi2"))) static uint64_t splitBy3(unsigned int a)
	{
		return _pdep_u64(static_cast<uint64_t>(a), 0x9249249249249249);
	}

	__attribute__((target("default"))) static unsigned int get3Bits(uint64_t code)
	{
		uint64_t a = code & 0x1249249249249249;
		a = (a ^ (a >> 2)) & 0x10c30c30c30c30c3;
		a = (a ^ (a >> 4)) & 0x100f00f00f00f00f;
		a = (a ^ (a >> 8)) & 0x1f0000ff0000ff;
		a = (a ^ (a >> 16)) & 0x1f00000000ffff;
		a = (a ^ a >> 32) & 0x1fffff;
		return static_cast<unsigned int>(a);
	}

	__attribute__((target("bmi2"))) static unsigned int get3Bits(uint64_t code)
	{
		return static_cast<unsigned int>(_pext_u64(code, 0x9249249249249249));
	}

private:
	// The Morton code
	uint64_t code_;
	// The depth of the Morton code
	unsigned int depth_;
};

using CodeSet = std::unordered_set<Code, Code::CodeHash>;
template <typename T>
using CodeMap = std::unordered_map<Code, T, Code::CodeHash>;
using CodeRay = std::vector<Code>;
}  // namespace ufomap

#endif  // UFOMAP_CODE_H