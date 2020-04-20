#ifndef UFOMAP_KEY_H
#define UFOMAP_KEY_H

#include <immintrin.h>  // x86intrin
#include <stdint.h>
#include <ufomap/types.h>

#include <array>
#include <cstddef>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ufomap
{
using KeyType = unsigned int;

/**
 * @brief A key represent an octree index at a specified depth
 *
 */
class Key
{
public:
	Key()
	{
	}

	Key(KeyType x, KeyType y, KeyType z, unsigned int depth)
		: key_{ x, y, z }, depth_(depth)
	{
	}

	Key(const Key& other) : key_{ other.key_ }, depth_(other.depth_)
	{
	}

	inline Key& operator=(const Key& rhs)
	{
		key_ = rhs.key_;
		depth_ = rhs.depth_;
		return *this;
	}

	/**
	 * @brief Get the depth that this key is specified at
	 *
	 * @return unsigned int The depth this key is specified at
	 */
	inline unsigned int getDepth() const
	{
		return depth_;
	}

	/**
	 * @brief Returns if this key is equal to another key at a certain depth
	 *
	 * @param other The other key to compare to
	 * @param depth The depth to do the comparison at
	 * @return true If the keys are equal at the specified depth
	 * @return false If the keys are not equal at the specified depth
	 */
	inline bool equals(const Key& other, unsigned int depth = 0) const
	{
		return (key_[0] >> depth) == (other.key_[0] >> depth) &&
					 (key_[1] >> depth) == (other.key_[1] >> depth) &&
					 (key_[2] >> depth) == (other.key_[2] >> depth);
	}

	inline bool operator==(const Key& rhs) const
	{
		return (key_[0] == rhs[0]) && (key_[1] == rhs[1]) && (key_[2] == rhs[2]) &&
					 (depth_ == rhs.depth_);
	}

	inline bool operator!=(const Key& rhs) const
	{
		return (key_[0] != rhs[0]) || (key_[1] != rhs[1]) || (key_[2] != rhs[2]) ||
					 (depth_ != rhs.depth_);
	}

	inline const KeyType& operator[](size_t index) const
	{
		return key_[index];
	}

	inline KeyType& operator[](size_t index)
	{
		return key_[index];
	}

	/**
	 * @brief Returns the x component of the key
	 *
	 * @return const KeyType& The x component of the key
	 */
	inline const KeyType& x() const
	{
		return key_[0];
	}

	/**
	 * @brief Returns the y component of the key
	 *
	 * @return const KeyType& The y component of the key
	 */
	inline const KeyType& y() const
	{
		return key_[1];
	}

	/**
	 * @brief Returns the z component of the key
	 *
	 * @return const KeyType& The z component of the key
	 */
	inline const KeyType& z() const
	{
		return key_[2];
	}

	/**
	 * @brief Returns the x component of the key
	 *
	 * @return KeyType& The x component of the key
	 */
	inline KeyType& x()
	{
		return key_[0];
	}

	/**
	 * @brief Returns the y component of the key
	 *
	 * @return KeyType& The y component of the key
	 */
	inline KeyType& y()
	{
		return key_[1];
	}

	/**
	 * @brief Returns the z component of the key
	 *
	 * @return KeyType& The z component of the key
	 */
	inline KeyType& z()
	{
		return key_[2];
	}

	/**
	 * @brief
	 *
	 */
	struct KeyHash
	{
		inline size_t operator()(const Key& key) const
		{
#if defined(__BMI2__) || defined(__AVX2__)  // TODO: Is correct?
			return _pdep_u64(static_cast<uint64_t>(key[0]), 0x9249249249249249) |
						 _pdep_u64(static_cast<uint64_t>(key[1]), 0x2492492492492492) |
						 _pdep_u64(static_cast<uint64_t>(key[2]), 0x4924924924924924);
#else
			return splitBy3(key[0]) | (splitBy3(key[1]) << 1) | (splitBy3(key[2]) << 2);
#endif
		}

	private:
#if !defined(__BMI2__) && !defined(__AVX2__)  // TODO: Is correct?
		inline uint64_t splitBy3(unsigned int a) const
		{
			uint64_t code = static_cast<uint64_t>(a) & 0x1fffff;
			code = (code | code << 32) & 0x1f00000000ffff;
			code = (code | code << 16) & 0x1f0000ff0000ff;
			code = (code | code << 8) & 0x100f00f00f00f00f;
			code = (code | code << 4) & 0x10c30c30c30c30c3;
			code = (code | code << 2) & 0x1249249249249249;
			return code;
		}
#endif
	};

private:
	// The key
	std::array<KeyType, 3> key_;
	// The depth of the key
	unsigned int depth_;
};

using KeySet = std::unordered_set<Key, Key::KeyHash>;
template <typename T>
using KeyMap = std::unordered_map<Key, T, Key::KeyHash>;
using KeyRay = std::vector<Key>;
}  // namespace ufomap

#endif  // UFOMAP_KEY_H