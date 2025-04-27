
#pragma once

#include <cstddef>

namespace fsb
{

/**
 * @defgroup Configuration Configuration settings
 * @brief Version information and configured size values for model parameters and buffers.
 * @{
 */

/**
 * @brief Configuration name
 */
static constexpr auto config_name = "default";

/**
 * @brief Library version
 */
static constexpr auto fsb_version = "0.0.1";

/**
 * @brief Max sizes configured for static memory allocation
 */
struct MaxSize
{
    /**
     * @brief Maximum number of bodies
     */
    static constexpr size_t bodies = 11U;

    /**
     * @brief Maximum number of joints
     */
    static constexpr size_t joints = 10U;

    /**
     * @brief Maximum number of generalized coordinates
     */
    static constexpr size_t coordinates = 15U;

    /**
     * @brief Maximum number of degrees of freedom
     */
    static constexpr size_t dofs = 12U;

    /**
     * @brief Linear algebra work vectors
    */
    static constexpr size_t linalg_work = 2048U;

    /**
     * @brief Maximum index size.
    */
    static constexpr size_t index = 65535U;

};

/**
 * @}
 */

} // namespace fsb
