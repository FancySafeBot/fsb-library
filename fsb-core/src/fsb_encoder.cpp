
#include <cstdint>
#include "fsb_encoder.h"
#include "fsb_types.h"

namespace fsb
{

EncoderStatus IncrementalEncoder::set_parameters(const uint8_t counter_resolution_bits, const uint32_t counts_per_rev,
                                                        const real_t value_per_rev)
{
    auto result = EncoderStatus::SUCCESS;
    if (counter_resolution_bits == 0U)
    {
        result = EncoderStatus::RESOLUTION_IS_ZERO;
    }
    else if (counter_resolution_bits > 32U)
    {
        result = EncoderStatus::RESOLUTION_EXCEEDS_MAX;
    }
    else if (counts_per_rev == 0U)
    {
        result = EncoderStatus::COUNTS_PER_REV_ZERO;
    }
    else if (value_per_rev < FSB_TOL)
    {
        result = EncoderStatus::VALUE_PER_REV_ZERO;
    }
    else
    {
        m_count = 0;
        m_value_offset = 0.0;
        m_value_per_rev = value_per_rev;
        m_counts_per_rev = counts_per_rev;

        // set resolution
        if (counter_resolution_bits == 32U)
        {
            m_counter_lower_limit = INT32_MIN;
        }
        else if (counter_resolution_bits == 16U)
        {
            m_counter_lower_limit = INT16_MIN;
        }
        else if (counter_resolution_bits == 8U)
        {
            m_counter_lower_limit = INT8_MIN;
        }
        else
        {
            m_counter_lower_limit = -1;
            for (uint8_t ind = 0U; ind < (counter_resolution_bits - 1U); ++ind)
            {
                m_counter_lower_limit *= 2;
            }
        }
    }

    return result;
}

void IncrementalEncoder::reset(const int32_t count, const real_t value)
{
    m_count = count;
    reset_value(value);
}

void IncrementalEncoder::reset_value(const real_t value)
{
    const real_t value_from_count = static_cast<real_t>(m_count) * m_value_per_rev / static_cast<real_t>(m_counts_per_rev);
    m_value_offset = value - value_from_count;
}

real_t IncrementalEncoder::decode(const int32_t count)
{
    // change in count
    int32_t delta = count - m_count;
    // unwrap logic
    if (delta < m_counter_lower_limit)
    {
        // wrapped around counting upwards
        delta = delta - 2 * m_counter_lower_limit;
    }
    else if (delta > -m_counter_lower_limit)
    {
        // wrapped around counting downwards
        delta = delta + 2 * m_counter_lower_limit;
    }
    else
    {
        // no wrapping
    }
    // save count
    m_count = count + delta;
    // convert counts to scaled value
    return m_value_offset + static_cast<real_t>(m_count) * m_value_per_rev / static_cast<real_t>(m_counts_per_rev);
}

int32_t IncrementalEncoder::encode(const real_t value)
{
    // update absolute counts
    const auto count = static_cast<int32_t>(value * static_cast<real_t>(m_counts_per_rev) / m_value_per_rev);
    // change in count
    const int32_t delta = count - m_count;
    // save count
    m_count = count;
    return delta;
}

}
