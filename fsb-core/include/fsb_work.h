
#ifndef FSB_WORK_H
#define FSB_WORK_H

#include <array>
#include <cstddef>
#include <cstdint>

namespace fsb
{

/**
 * @defgroup Work Work buffers
 * @brief Fixed-size work buffer allocation utilities.
 *
 * This module provides a small, deterministic allocator that hands out
 * pointers to contiguous ranges within a fixed-size private array.
 * allocations must be freed in strict LIFO order, and no dynamic memory
 * allocation is performed.
 *
 * This is useful for managing temporary buffers within linear algebra
 * operations without dynamic memory allocation.
 *
 * @{
 */

enum class WorkArrayStatus : uint8_t
{
	/** @brief Successful operation */
	SUCCESS = 0,
	/** @brief Not enough space available */
	FULL,
	/** @brief Invalid argument (null pointer, out of range, size==0, etc.) */
	INVALID_ARGUMENT,
	/** @brief Attempted to free memory out of LIFO order */
	OUT_OF_ORDER
};

/**
 * @brief A view describing an allocated work block.
 */
template <typename T> struct WorkBlock
{
	T* data = nullptr;
	size_t size = 0;
	size_t offset = 0;

	[[nodiscard]] explicit operator bool() const noexcept
	{
		return (data != nullptr) && (size > 0U);
	}
};

/**
 * @brief Fixed-capacity allocator for temporary work buffers.
 *
 * Allocations are contiguous ranges within an internal `std::array<T, Capacity>`.
 * This class performs no dynamic allocation.
 */
template <typename T, size_t Capacity> class WorkArray final
{
public:
	WorkArray() = default;

	static constexpr size_t get_capacity() noexcept
	{
		return Capacity;
	}

	[[nodiscard]] size_t get_used() const noexcept
	{
		return m_top;
	}

	[[nodiscard]] size_t get_remaining() const noexcept
	{
		return Capacity - get_used();
	}

	[[nodiscard]] T* data() noexcept
	{
		return m_data.data();
	}

	[[nodiscard]] const T* data() const noexcept
	{
		return m_data.data();
	}

	/**
	 * @brief Reset allocator state (frees all allocations).
	 */
	void reset() noexcept
	{
		m_top = 0U;
	}

	/**
	 * @brief Get a marker representing the current top of stack.
	 */
	[[nodiscard]] size_t get_marker() const noexcept
	{
		return m_top;
	}

	/**
	 * @brief Restore the allocator to a previous marker (LIFO bulk free).
	 */
	WorkArrayStatus restore(size_t marker) noexcept
	{
		if (marker > m_top)
		{
			return WorkArrayStatus::INVALID_ARGUMENT;
		}
		m_top = marker;
		return WorkArrayStatus::SUCCESS;
	}

	/**
	 * @brief Allocate a contiguous work block of `len` elements.
	 *
	 * @param[in]  len Length (in elements) to allocate.
	 * @param[out] out Filled with pointer+length on success.
	 * @return Status of the allocation.
	 */
	WorkArrayStatus allocate(size_t len, WorkBlock<T>& out) noexcept
	{
		out = {};
		if ((len == 0U) || (len > Capacity))
		{
			return WorkArrayStatus::INVALID_ARGUMENT;
		}
		if ((m_top + len) > Capacity)
		{
			return WorkArrayStatus::FULL;
		}

		out.offset = m_top;
		out.data = &m_data[m_top];
		out.size = len;
		m_top += len;
		return WorkArrayStatus::SUCCESS;
	}

	/**
	 * @brief Deallocate a work block previously returned by allocate().
	 *
	 * @param[in] ptr Pointer returned by allocate().
	 * @param[in] len Length originally allocated.
	 * @return Status of the deallocation.
	 */
	WorkArrayStatus deallocate(T* ptr, size_t len) noexcept
	{
		if ((ptr == nullptr) || (len == 0U) || (len > Capacity))
		{
			return WorkArrayStatus::INVALID_ARGUMENT;
		}

		T* const begin = m_data.data();
		T* const end = m_data.data() + Capacity;
		if ((ptr < begin) || (ptr >= end))
		{
			return WorkArrayStatus::INVALID_ARGUMENT;
		}

		const size_t start = static_cast<size_t>(ptr - begin);
		if ((start + len) > Capacity)
		{
			return WorkArrayStatus::INVALID_ARGUMENT;
		}

		// Strict LIFO: must free from the current top of stack.
		if ((start + len) != m_top)
		{
			return WorkArrayStatus::OUT_OF_ORDER;
		}

		m_top = start;
		return WorkArrayStatus::SUCCESS;
	}

	/**
	 * @brief Deallocate a work block.
	 */
	WorkArrayStatus deallocate(const WorkBlock<T>& block) noexcept
	{
		return deallocate(block.data, block.size);
	}

private:
	std::array<T, Capacity> m_data = {};
	size_t m_top = 0U;
};

/**
 * @brief RAII helper that restores a WorkArray to its entry marker.
 *
 * Useful for allocating multiple temporary buffers and guaranteeing they
 * are released even when returning early.
 */
template <typename T, size_t Capacity> class WorkFrame final
{
public:
	explicit WorkFrame(WorkArray<T, Capacity>& work) noexcept
		: m_work(work)
		, m_marker(work.get_marker())
	{
	}

	WorkFrame(const WorkFrame&) = delete;
	WorkFrame& operator=(const WorkFrame&) = delete;
	WorkFrame(WorkFrame&&) = delete;
	WorkFrame& operator=(WorkFrame&&) = delete;

	~WorkFrame() noexcept
	{
		(void)m_work.restore(m_marker);
	}

	[[nodiscard]] size_t get_marker() const noexcept
	{
		return m_marker;
	}

	WorkArrayStatus allocate(size_t len, WorkBlock<T>& out) noexcept
	{
		return m_work.allocate(len, out);
	}

private:
	WorkArray<T, Capacity>& m_work;
	size_t m_marker;
};

/**
 * @}
 */

} // namespace fsb

#endif // FSB_WORK_H

