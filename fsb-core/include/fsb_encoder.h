
#pragma once

#include <cstdint>
#include "fsb_types.h"

namespace fsb
{

/**
 * @defgroup Encoder Incremental Encoder
 * @brief Decode and encode incremental encoder counts and position.
 *
 * @{
 */

/**
 * @brief Status from setting encoder parameters
 *
 */
enum class EncoderStatus
{
    /**
     * @brief Parameters applied successfully
     */
    SUCCESS,
    /**
     * @brief Resolution of encoder (in number of bits) must be non-zero
     */
    RESOLUTION_IS_ZERO,
    /**
     * @brief Encoder resolution exceeds maximum of 32 bits
     */
    RESOLUTION_EXCEEDS_MAX,
    /**
     * @brief Counts per revolution must be non-zero
     */
    COUNTS_PER_REV_ZERO,
    /**
     * @brief Value per revolution is below tolerance
     *
     */
    VALUE_PER_REV_ZERO
};

/**
 * @brief Incremental encoder
 *
 */
class IncrementalEncoder
{
public:
    IncrementalEncoder() = default;

    /**
     * @brief Set encoder parameters
     *
     * @param counter_resolution_bits Bit depth of counter (max 64)
     * @param counts_per_rev Number of counts per revolution
     * @param value_per_rev scaled units per revolution
     * @return Encoder status
     */
    EncoderStatus
    set_parameters(uint8_t counter_resolution_bits, uint32_t counts_per_rev, real_t value_per_rev);

    /**
     * @brief Reset encoder to counts and associated value
     *
     * @param count Number of counts
     * @param value Value to reset
     */
    void reset(int32_t count, real_t value);

    /**
     * @brief Reset value at current count
     *
     * @param value Value to reset
     */
    void reset_value(real_t value);

    /**
     * @brief Get value in scaled units from counter update
     *
     * @param count new input from counter
     * @return real_t scaled value
     */
    real_t decode(int32_t count);

    /**
     * @brief encode value to number of counts
     *
     * @param value value to encode
     * @return change in counts from last encode update
     */
    int32_t encode(real_t value);

private:
    // encoder count
    int32_t m_count = 0;
    // value offset at count == 0
    real_t m_value_offset = 0.0;
    // scaled units per revolution
    real_t m_value_per_rev = 0.0;
    // Encoder counts per revolution
    uint32_t m_counts_per_rev = 0U;
    // incremental encoder limit based on counter bit depth
    int32_t m_counter_lower_limit = 0;
};

/**
 * @}
 */

} // namespace fsb
