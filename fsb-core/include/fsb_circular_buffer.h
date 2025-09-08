#pragma once

#include <array>
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

enum class CircularBufferStatus : uint8_t
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
    CircularBuffer() = default;

    /**
     * @brief Add value to the buffer if there is space available.
     *
     * @param push_value New value to add to buffer.
     * @return Status of operation.
     */
    CircularBufferStatus push(BufferType push_value);

    /**
     * @brief Add value to buffer and overwrite oldest value if buffer is full.
     *
     * @param push_value New value to add to buffer.
     * @return Status of operation.
     */
    CircularBufferStatus force_push(BufferType push_value);

    /**
     * @brief Get oldest value from buffer.
     *
     * @param popped_value Oldest value in buffer.
     * @return Status of operation.
     */
    CircularBufferStatus pop(BufferType &popped_value);

    /**
     * @brief Get oldest value from buffer.
     *
     * @param popped_values All values in buffer.
     * @return Status of operation.
     */
    CircularBufferStatus pop_all(std::array<BufferType, BufferSize> &popped_values, size_t& num_popped);

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
            filled_size = BufferSize;
        }
        else
        {
            filled_size = (m_tail > m_head ? (BufferSize - m_tail) + m_head : m_head - m_tail);
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
            remaining_size = (m_tail > m_head ? m_tail - m_head : BufferSize - (m_head - m_tail));
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
        return BufferSize;
    }

   private:
    /**
     * @brief Circular buffer of commands for stepper
     */
    std::array<BufferType, BufferSize> m_buffer;
    /**
     * @brief Index of oldest filled buffer position
     *
     * Buffer is empty if @c m_tail is equal to @c m_head
     */
    size_t m_tail = 0;
    /**
     * @brief Index of latest open buffer position
     *
     * Buffer is empty if @c m_tail is equal to @c m_head
     */
    size_t m_head = 0;
    /**
     * @brief `true` if buffer is full, `false` otherwise
     */
    bool m_full = false;
};

// ===============================
// CircularBuffer Implementation
// ===============================

template<typename BufferType, size_t BufferSize>
inline CircularBufferStatus CircularBuffer<BufferType, BufferSize>::push(BufferType push_value)
{
    CircularBufferStatus status = CircularBufferStatus::SUCCESS;
    if (GetRemaining() == 0)
    {
        status = CircularBufferStatus::FULL;
    }
    else
    {
        // push to head of queue
        m_buffer[m_head] = push_value;
        m_head = (m_head + 1) % BufferSize;
        m_full = (m_head == m_tail);
    }
    return status;
}

template<typename BufferType, size_t BufferSize>
inline CircularBufferStatus CircularBuffer<BufferType, BufferSize>::force_push(BufferType push_value)
{
    CircularBufferStatus status = CircularBufferStatus::SUCCESS;
    if (GetRemaining() == 0)
    {
        m_buffer[m_head] = push_value;
        m_head = (m_head + 1) % BufferSize;
        m_tail = m_head;
        status = CircularBufferStatus::OVERWRITE;
    }
    else
    {
        // push to head of queue
        m_buffer[m_head] = push_value;
        m_head = (m_head + 1) % BufferSize;
        m_full = (m_head == m_tail);
    }
    return status;
}

template<typename BufferType, size_t BufferSize>
inline CircularBufferStatus CircularBuffer<BufferType, BufferSize>::pop(BufferType &popped_value)
{
    CircularBufferStatus status = CircularBufferStatus::SUCCESS;
    if (GetFilled() == 0)
    {
        status = CircularBufferStatus::EMPTY;
    }
    else
    {
        // consume at tail of queue
        popped_value = m_buffer[m_tail];
        // advance
        m_tail = (m_tail + 1) % BufferSize;
        m_full = false;
    }
    return status;
}

template<typename BufferType, size_t BufferSize>
CircularBufferStatus CircularBuffer<BufferType, BufferSize>::pop_all(std::array<BufferType, BufferSize> &popped_values, size_t& num_popped)
{
    num_popped = 0;
    while (GetFilled() > 0)
    {
        // consume at tail of queue
        popped_values[num_popped] = m_buffer[m_tail];
        num_popped++;
        // advance
        m_tail = (m_tail + 1) % BufferSize;
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
