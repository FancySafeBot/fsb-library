
#ifndef FSB_ENCODER_H
#define FSB_ENCODER_H

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
enum class EncoderStatus : uint8_t
{
    /**
     * @brief Parameters applied successfully
     */
    SUCCESS = 0,
    /**
     * @brief Resolution of encoder (in number of bits) must be non-zero
     */
    RESOLUTION_IS_ZERO = 1,
    /**
     * @brief Encoder resolution exceeds maximum of 32 bits
     */
    RESOLUTION_EXCEEDS_MAX = 2,
    /**
     * @brief Counts per revolution must be non-zero
     */
    COUNTS_PER_REV_ZERO = 3,
    /**
     * @brief Value per revolution is below tolerance
     *
     */
    VALUE_PER_REV_ZERO = 4
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
    set_parameters(uint8_t counter_resolution_bits, uint32_t counts_per_rev, Real value_per_rev);

    /**
     * @brief Reset encoder to counts and associated value
     *
     * @param count Number of counts
     * @param value Value to reset
     */
    void reset(int32_t count, Real value);

    /**
     * @brief Reset value at current count
     *
     * @param value Value to reset
     */
    void reset_value(Real value);

    /**
     * @brief Get value in scaled units from counter update
     *
     * @param count New input from counter
     * @return Scaled value in user-defined units
     */
    Real decode(int32_t count);

    /**
     * @brief Encode value to number of counts
     *
     * @param value Value to encode
     * @return Change in counts from last encode update
     */
    int32_t encode(Real value);

private:
    // encoder count
    int32_t m_count = 0;
    // value offset at count == 0
    Real m_value_offset = 0.0;
    // scaled units per revolution
    Real m_value_per_rev = 0.0;
    // Encoder counts per revolution
    uint32_t m_counts_per_rev = 0U;
    // incremental encoder limit based on counter bit depth
    int32_t m_counter_lower_limit = 0;
};

/**
 * @}
 */

} // namespace fsb

#endif
