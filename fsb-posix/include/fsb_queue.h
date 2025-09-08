#pragma once

#include <cstdlib>
#include <cstdint>
#include <array>
#include <pthread.h>

#include "fsb_thread.h"
#include "fsb_circular_buffer.h"

namespace fsb
{

/**
 * @defgroup   Queue Queue
 * @brief      Queue
 *
 * @{
 *
 */

enum class QueueStatus
{
    SUCCESS,
    UNINITIALIZED,
    FULL,
    OVERWRITE,
    TIMEOUT,
    ERROR
};

/**
 * @brief Queue
 *
 */
template <typename QueueType, size_t QueueSize> class Queue
{
public:
    Queue();
    ~Queue();

    // Delete copy constructor and copy assignment operator
    Queue(const Queue&) = delete;
    Queue& operator=(const Queue&) = delete;

    // Delete move constructor and move assignment operator
    Queue(Queue&&) = delete;
    Queue& operator=(Queue&&) = delete;

    /**
     * @brief Add value to buffer if there is space available.
     *
     * @param push_value New value to add to buffer.
     * @return Status of operation.
     */
    QueueStatus Push(QueueType push_value);

    /**
     * @brief Add value to buffer and overwrite oldest value if buffer is full.
     *
     * @param push_value New value to add to buffer.
     * @return Status of operation.
     */
    QueueStatus ForcePush(QueueType push_value);

    /**
     * @brief Get oldest value from buffer.
     *
     * @param popped_value Oldest value in buffer.
     * @return Status of operation.
     */
    QueueStatus Pop(QueueType& popped_value);

    /**
     * @brief Wait for a new value to be added to the buffer, then pop all values.
     *
     * @param popped_values All values removed from buffer.
     * @param num_popped Number of values removed from buffer.
     * @return Status of operation.
     */
    QueueStatus PopAll(std::array<QueueType, QueueSize>& popped_values, size_t& num_popped);

    /**
     * @brief Wait for a new value to be added to the buffer, then pop all values.
     *
     * @param popped_values All values removed from buffer.
     * @param num_popped Number of values removed from buffer.
     * @param timeout Maximum time to wait for new data.
     * @return Status of operation.
     */
    QueueStatus PopWait(
        std::array<QueueType, QueueSize>& popped_values, size_t& num_popped,
        const struct timespec& timeout);

    /**
     * @brief Reset buffer to empty state.
     *
     */
    QueueStatus Reset();

private:
    bool                                 m_initialized = false;
    pthread_mutex_t                      m_mutex = {};
    pthread_cond_t                       m_cond_var = {};
    CircularBuffer<QueueType, QueueSize> m_buffer = {};
};

// ===============================
// Queue Implementation
// ===============================

template <typename QueueType, size_t QueueSize> inline Queue<QueueType, QueueSize>::Queue()
{
    if (mutex_initialize(m_mutex) == LockStatus::SUCCESS)
    {
        if (condvar_initialize(m_cond_var) == LockStatus::SUCCESS)
        {
            m_initialized = true;
        }
        else
        {
            mutex_destroy(m_mutex);
        }
    }
}

template <typename QueueType, size_t QueueSize> Queue<QueueType, QueueSize>::~Queue()
{
    if (m_initialized)
    {
        mutex_destroy(m_mutex);
        condvar_destroy(m_cond_var);
        m_initialized = false;
    }
}

template <typename QueueType, size_t QueueSize>
inline QueueStatus Queue<QueueType, QueueSize>::Push(const QueueType push_value)
{
    if (!m_initialized)
    {
        return QueueStatus::UNINITIALIZED;
    }
    if (mutex_lock(m_mutex) != LockStatus::SUCCESS)
    {
        return QueueStatus::ERROR;
    }
    CircularBufferStatus buf_status = m_buffer.push(push_value);
    mutex_unlock(m_mutex);
    condvar_signal(m_cond_var);
    if (buf_status == CircularBufferStatus::FULL)
    {
        return QueueStatus::FULL;
    }
    else if (buf_status != CircularBufferStatus::SUCCESS)
    {
        return QueueStatus::ERROR;
    }
    return QueueStatus::SUCCESS;
}

template <typename QueueType, size_t QueueSize>
inline QueueStatus Queue<QueueType, QueueSize>::ForcePush(QueueType push_value)
{
    if (!m_initialized)
    {
        return QueueStatus::UNINITIALIZED;
    }
    if (mutex_lock(m_mutex) != LockStatus::SUCCESS)
    {
        return QueueStatus::ERROR;
    }
    CircularBufferStatus buf_status = m_buffer.force_push(push_value);
    mutex_unlock(m_mutex);
    condvar_signal(m_cond_var);
    if (buf_status == CircularBufferStatus::OVERWRITE)
    {
        return QueueStatus::OVERWRITE;
    }
    else if (buf_status != CircularBufferStatus::SUCCESS)
    {
        return QueueStatus::ERROR;
    }
    return QueueStatus::SUCCESS;
}

template <typename QueueType, size_t QueueSize>
inline QueueStatus Queue<QueueType, QueueSize>::PopAll(
    std::array<QueueType, QueueSize>& popped_values, size_t& num_popped)
{
    if (!m_initialized)
    {
        return QueueStatus::UNINITIALIZED;
    }
    if (mutex_lock(m_mutex) != LockStatus::SUCCESS)
    {
        return QueueStatus::ERROR;
    }
    CircularBufferStatus buf_status = m_buffer.pop_all(popped_values, num_popped);
    mutex_unlock(m_mutex);
    return (
        buf_status == CircularBufferStatus::SUCCESS ? QueueStatus::SUCCESS : QueueStatus::ERROR);
}

template <typename QueueType, size_t QueueSize>
inline QueueStatus Queue<QueueType, QueueSize>::PopWait(
    std::array<QueueType, QueueSize>& popped_values, size_t& num_popped,
    const struct timespec& timeout)
{
    num_popped = 0;
    if (!m_initialized)
    {
        return QueueStatus::UNINITIALIZED;
    }
    if (mutex_lock(m_mutex) != LockStatus::SUCCESS)
    {
        return QueueStatus::ERROR;
    }
    // pop all values
    m_buffer.pop_all(popped_values, num_popped);
    if (num_popped > 0)
    {
        // if we already have data, return it immediately
        mutex_unlock(m_mutex);
        return QueueStatus::SUCCESS;
    }
    // wait for new data
    const LockStatus cv_status = condvar_wait_timeout(m_cond_var, m_mutex, timeout);
    QueueStatus result = QueueStatus::SUCCESS;
    if (cv_status != LockStatus::SUCCESS)
    {
        result = (cv_status == LockStatus::TIMEOUT ? QueueStatus::TIMEOUT : QueueStatus::ERROR);
    }
    else
    {
        // pop all values
        m_buffer.pop_all(popped_values, num_popped);
    }
    mutex_unlock(m_mutex);
    return result;
}

template <typename QueueType, size_t QueueSize>
inline QueueStatus Queue<QueueType, QueueSize>::Pop(QueueType& popped_value)
{
    if (!m_initialized)
    {
        return QueueStatus::UNINITIALIZED;
    }
    if (mutex_lock(m_mutex) != LockStatus::SUCCESS)
    {
        return QueueStatus::ERROR;
    }
    CircularBufferStatus buf_status = m_buffer.pop(popped_value);
    mutex_unlock(m_mutex);
    return (
        buf_status == CircularBufferStatus::SUCCESS ? QueueStatus::SUCCESS : QueueStatus::ERROR);
}

template <typename QueueType, size_t QueueSize>
inline QueueStatus Queue<QueueType, QueueSize>::Reset()
{
    if (!m_initialized)
    {
        return QueueStatus::UNINITIALIZED;
    }
    if (mutex_lock(m_mutex) != LockStatus::SUCCESS)
    {
        return QueueStatus::ERROR;
    }
    m_buffer.Reset();
    mutex_unlock(m_mutex);
    return QueueStatus::SUCCESS;
}

// template<typename QueueType, size_t QueueSize>
// inline QueueStatus Queue<QueueType, QueueSize>::Push(const QueueType push_value)
// {
//     QueueStatus status = QueueStatus::SUCCESS;
//     {
//         std::lock_guard<std::mutex> lock(m_mutex);
//         CircularBufferStatus buf_status = m_buffer.Push(push_value);
//         if (buf_status == CircularBufferStatus::FULL) {
//             status = QueueStatus::FULL;
//         } else if (buf_status != CircularBufferStatus::SUCCESS) {
//             status = QueueStatus::ERROR;
//         }
//     }
//     m_cond_var.notify_all();
//     return status;
// }

// template<typename QueueType, size_t QueueSize>
// inline QueueStatus Queue<QueueType, QueueSize>::ForcePush(QueueType push_value)
// {
//     QueueStatus status = QueueStatus::SUCCESS;
//     {
//         std::lock_guard<std::mutex> lock(m_mutex);
//         CircularBufferStatus buf_status = m_buffer.ForcePush(push_value);
//         if (buf_status == CircularBufferStatus::OVERWRITE) {
//             status = QueueStatus::OVERWRITE;
//         } else if (buf_status != CircularBufferStatus::SUCCESS) {
//             status = QueueStatus::ERROR;
//         }
//     }
//     m_cond_var.notify_all();
//     return status;
// }

// template<typename QueueType, size_t QueueSize>
// inline QueueStatus Queue<QueueType, QueueSize>::PopAll(std::array<QueueType, QueueSize>
// &popped_values,
//     size_t& num_popped, const struct timespec& timeout)
// {
//     std::unique_lock<std::mutex> lock(m_mutex);
//     std::cv_status status = m_cond_var.wait_for(lock, std::chrono::nanoseconds(timeout.tv_nsec) +
//     std::chrono::seconds(timeout.tv_sec), [this] { return !m_buffer.Empty(); }); if (status ==
//     std::cv_status::timeout)
//     {
//         return QueueStatus::TIMEOUT;
//     }
//     // pop all values
//     num_popped = 0;
//     while (!m_buffer.Empty())
//     {
//         QueueType value;
//         if (m_buffer.Pop(value) == QueueStatus::SUCCESS)
//         {
//             popped_values[num_popped++] = value;
//         }
//     }
//     return QueueStatus::SUCCESS;
// }

// template<typename QueueType, size_t QueueSize>
// inline QueueStatus Queue<QueueType, QueueSize>::Pop(QueueType &popped_value)
// {
//     std::lock_guard<std::mutex> lock(m_mutex);
//     CircularBufferStatus buf_status = m_buffer.Pop(popped_value);
//     return buf_status == CircularBufferStatus::SUCCESS ? QueueStatus::SUCCESS :
//     QueueStatus::ERROR;
// }

// template<typename QueueType, size_t QueueSize>
// inline void Queue<QueueType, QueueSize>::Reset()
// {
//     std::lock_guard<std::mutex> lock(m_mutex);
//     m_buffer.Reset();
// }

} // namespace fsb
