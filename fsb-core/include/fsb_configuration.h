
#ifndef FSB_CONFIGURATION_H
#define FSB_CONFIGURATION_H

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
static constexpr auto kConfigName = "default";

/**
 * @brief Library version
 */
static constexpr auto kFsbVersion = "0.0.1";

/**
 * @brief Max sizes configured for static memory allocation
 */
struct MaxSize
{
    /**
     * @brief Maximum number of bodies
     */
    static constexpr size_t kBodies = 11U;

    /**
     * @brief Maximum number of joints
     */
    static constexpr size_t kJoints = 10U;

    /**
     * @brief Maximum number of generalized coordinates
     */
    static constexpr size_t kCoordinates = 15U;

    /**
     * @brief Maximum number of degrees of freedom
     */
    static constexpr size_t kDofs = 12U;

    /**
     * @brief Linear algebra work vectors
    */
    static constexpr size_t kLinalgWork = 2048U;

    /**
     * @brief Maximum index size.
    */
    static constexpr size_t kIndex = 65535U;

    /**
     * @brief Maximum thread execution priority
    */
    static constexpr int kPriority = 98;
};

/**
 * @}
 */

} // namespace fsb

#endif // FSB_CONFIGURATION_H
