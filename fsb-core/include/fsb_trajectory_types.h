#ifndef FSB_TRAJECTORY_TYPES_H
#define FSB_TRAJECTORY_TYPES_H

#include "fsb_types.h"
#include "fsb_motion.h"

namespace fsb {

/**
 * @defgroup TopicTrajectorySegment Small segment trajectories
 * @brief Segments with motion constraints
 * @{
 */

/**
 * @brief Trajectory scalar motion state
 */
struct TrajState {
    /**
     * @brief Position
     */
    real_t position;
    /**
     * @brief Velocity
     */
    real_t velocity;
    /**
     * @brief Acceleration
     */
    real_t acceleration;
    /**
     * @brief Jerk
     */
    real_t jerk;
};

/**
 * @brief Trajectory vector motion state
 */
struct TrajState3 {
    /**
     * @brief Position
     */
    Vec3 position;
    /**
     * @brief Velocity
     */
    Vec3 velocity;
    /**
     * @brief Acceleration
     */
    Vec3 acceleration;
    /**
     * @brief Jerk
     */
    Vec3 jerk;
};

/**
 * @}
 */

}

#endif
