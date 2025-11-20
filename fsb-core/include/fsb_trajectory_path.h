#ifndef FSB_TRAJECTORY_PATH_H
#define FSB_TRAJECTORY_PATH_H

#include "fsb_trajectory_types.h"
#include "fsb_types.h"

#include "fsb_motion.h"

namespace fsb
{

/**
 * @defgroup TopicPath Cartesian Path
 * @brief Path trajectories in Cartesian space
 * @{
 */

struct PathPoint
{
    Transform pose = transform_identity();
    Real      displacement = 0.0;
    Vec3      direction = {};
};

struct PathPva
{
    Real position = 0.0;
    Real velocity = 0.0;
    Real acceleration = 0.0;
};

class PathSegment : public Segment6
{
public:
    /**
     * @brief Evaluate the segment at a given time.
     * @param[in] t_eval Time at which to evaluate the segment.
     * @return Cartesian position, velocity, and acceleration at the given time.
     */
    CartesianPva evaluate(Real t_eval) const final;

    /**
     *
     * @param displacement Position, velocity and acceleration along path
     * @return Cartesian pose, velocity, and acceleration at the given displacement along the path
     */
    CartesianPva evaluate_path(const PathPva& displacement);

    /**
     * @brief Get final state of the segment.
     * @return Final Cartesian position, velocity, and acceleration at the end of the segment.
     */
    CartesianPva get_final_state() const final;

private:
    CartesianPva m_initial = {};
    CartesianPva m_final = {};

    Real m_start_displacement = 0.0;
    Real m_arc_length = 0.0;
};

class PathTrajectory : public Segment6
{};

/**
 * @}
 */

} // namespace fsb

#endif // FANCYSAFEBOT_FSB_PATH_H
