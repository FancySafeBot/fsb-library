#pragma once

#include <cstdlib>
#include <cstdint>

namespace fsb
{

/**
 * @defgroup   CircularBuffer Circular buffer
 * @brief      Circular buffer
 *
 * @{
 *
 */

enum class CircularBufferStatus
{
    /**
     * @brief Successful operation
     */
    SUCCESS = 0,
    /**
     * @brief Operation failed, buffer is full
     */
    FULL,
    /**
     * @brief Operation failed, buffer is empty
     */
    EMPTY,
    /**
     * @brief Adding to buffer overwrote existing data
     */
    OVERWRITE
};

/**
 * @brief Circular Buffer
 *
 */
template<typename BufferType, size_t BufferSize>
class CircularBuffer
{
   public:
    CircularBuffer() : m_buffer(), m_tail(0), m_head(0), m_size(BufferSize), m_full(false) {}

    /**
     * @brief Add value to buffer if there is space available.
     *
     * @param push_value New value to add to buffer.
     * @return Status of operation.
     */
    CircularBufferStatus Push(BufferType push_value);

    /**
     * @brief Add value to buffer and overwrite oldest value if buffer is full.
     *
     * @param push_value New value to add to buffer.
     * @return Status of operation.
     */
    CircularBufferStatus ForcePush(BufferType push_value);

    /**
     * @brief Get oldest value from buffer.
     *
     * @param popped_value Oldest value in buffer.
     * @return Status of operation.
     */
    CircularBufferStatus Pop(BufferType &popped_value);

    /**
     * @brief Get oldest value from buffer.
     *
     * @param popped_values All values in buffer.
     * @return Status of operation.
     */
    CircularBufferStatus PopAll(std::array<BufferType, BufferSize> &popped_values, size_t& num_popped);

    /**
     * @brief Reset buffer to empty state.
     *
     */
    void Reset();

    /**
     * @brief Get number of filled buffer positions.
     *
     * @return Number of filled buffer positions.
     */
    size_t GetFilled() const
    {
        size_t filled_size = 0;
        if (m_full)
        {
            filled_size = m_size;
        }
        else
        {
            filled_size = (m_tail > m_head ? (m_size - m_tail) + m_head : m_head - m_tail);
        }
        return filled_size;
    }

    /**
     * @brief Get number of remaining buffer positions.
     *
     * @return Number of remaining buffer positions.
     */
    size_t GetRemaining() const
    {
        size_t remaining_size = 0;
        if (m_full)
        {
            remaining_size = 0;
        }
        else
        {
            remaining_size = (m_tail > m_head ? m_tail - m_head : m_size - (m_head - m_tail));
        }
        return remaining_size;
    }

    /**
     * @brief Get total buffer size.
     *
     * @return Total buffer size.
     */
    size_t GetSize() const
    {
        return m_size;
    }

   private:
    /**
     * @brief Circular buffer of commands for stepper
     */
    BufferType m_buffer[BufferSize];
    /**
     * @brief Index of oldest filled buffer position
     *
     * Buffer is empty if @c m_tail is equal to @c m_head
     */
    size_t m_tail;
    /**
     * @brief Index of latest open buffer position
     *
     * Buffer is empty if @c m_tail is equal to @c m_head
     */
    size_t m_head;
    /**
     * @brief Total buffer size
     */
    size_t m_size;
    /**
     * @brief `true` if buffer is full, `false` otherwise
     */
    bool m_full;
};

// ===============================
// CircularBuffer Implementation
// ===============================

template<typename BufferType, size_t BufferSize>
inline CircularBufferStatus CircularBuffer<BufferType, BufferSize>::Push(BufferType push_value)
{
    CircularBufferStatus status = CircularBufferStatus::SUCCESS;
    const size_t remaining_size = GetRemaining();
    if (remaining_size == 0)
    {
        status = CircularBufferStatus::FULL;
    }
    else
    {
        // push to head of queue
        m_buffer[m_head] = push_value;
        m_head = (m_head + 1) % m_size;
        m_full = (m_head == m_tail);
    }
    return status;
}

template<typename BufferType, size_t BufferSize>
inline CircularBufferStatus CircularBuffer<BufferType, BufferSize>::ForcePush(BufferType push_value)
{
    CircularBufferStatus status = CircularBufferStatus::SUCCESS;
    const size_t remaining_size = GetRemaining();
    if (remaining_size == 0)
    {
        m_buffer[m_head] = push_value;
        m_head = (m_head + 1) % m_size;
        m_tail = m_head;
        status = CircularBufferStatus::OVERWRITE;
    }
    else
    {
        // push to head of queue
        m_buffer[m_head] = push_value;
        m_head = (m_head + 1) % m_size;
        m_full = (m_head == m_tail);
    }
    return status;
}

template<typename BufferType, size_t BufferSize>
inline CircularBufferStatus CircularBuffer<BufferType, BufferSize>::Pop(BufferType &popped_value)
{
    CircularBufferStatus status = CircularBufferStatus::SUCCESS;
    const size_t filled_size = GetFilled();
    if (filled_size == 0)
    {
        status = CircularBufferStatus::EMPTY;
    }
    else
    {
        // consume at tail of queue
        popped_value = m_buffer[m_tail];
        // advance
        m_tail = (m_tail + 1) % m_size;
        m_full = false;
    }
    return status;
}

template<typename BufferType, size_t BufferSize>
CircularBufferStatus CircularBuffer<BufferType, BufferSize>::PopAll(std::array<BufferType, BufferSize> &popped_values, size_t& num_popped)
{
    num_popped = 0;
    while (GetFilled() > 0)
    {
        // consume at tail of queue
        popped_values[num_popped] = m_buffer[m_tail];
        num_popped++;
        // advance
        m_tail = (m_tail + 1) % m_size;
        m_full = false;
    }
    return CircularBufferStatus::SUCCESS;
}


template<typename BufferType, size_t BufferSize>
inline void CircularBuffer<BufferType, BufferSize>::Reset()
{
    m_head = 0;
    m_tail = 0;
    m_full = false;
}

}