
#ifndef FSB_BODY_H
#define FSB_BODY_H

#include <array>
#include <cstddef>
#include "fsb_configuration.h"
#include "fsb_quaternion.h"
#include "fsb_motion.h"
#include "fsb_types.h"

namespace fsb
{

/**
 * @defgroup TopicBody Rigid Body
 * @brief Rigid body (link) mass properties and transforms
 *
 * @{
 */

/**
 * @brief Inertia tensor
 */
struct Inertia
{
    /** @brief Ixx component of tensor */
    real_t ixx;
    /** @brief Ixx component of tensor */
    real_t iyy;
    /** @brief Izz component of tensor */
    real_t izz;
    /** @brief Ixy component of tensor */
    real_t ixy;
    /** @brief Ixz component of tensor */
    real_t ixz;
    /** @brief Iyz component of tensor */
    real_t iyz;
};

/**
 * @brief Mass properties of a body
 */
struct MassProps
{
    /**
     * @brief Mass of body
     */
    real_t mass;
    /**
     * @brief Center of mass
     */
    Vec3 com;
    /**
     * @brief Inertia tensor
     */
    Inertia inertia;
};

/**
 * @brief Principal inertia of body
 */
struct PrincipalInertia
{
    /**
     * @brief Orientation of principal inertia axes
     */
    Quaternion rot;
    /**
     * @brief Principal inertia values. Ixy, Ixz, and Iyz are zero.
     */
    Inertia inertia;
};

/**
 * @brief Container of Cartesian pose, velocity, and acceleration for a list of bodies
 */
struct BodyCartesianPva
{
    /**
     * @brief List of bodies with Cartesian pose, velocity, and acceleration.
     */
    std::array<CartesianPva, MaxSize::bodies> body;
};

/**
 * @brief Link body parameters
 */
struct Body
{
    /**
     * @brief Transform offset of from nominal body origin
     */
    MotionVector origin_offset;
    /**
     * @brief Mass properties of body
     */
    MassProps mass_props;
    /**
     * @brief Principal inertia of body
     */
    PrincipalInertia principal_inertia;
    /**
     * @brief Index of parent joint
     */
    size_t joint_index;
};

/**
 * @brief Rotate inertia tensor
 *
 * \f$ \mathbf{I}_0 = \mathbf{R}_{0a} \mathcal{I}_a \mathbf{R}^T_{0a} \f$
 *
 * @param[in] rot Rotation quaternion
 * @param[in] inertia Inertia tensor
 * @return Rotated inertia tensor
 */
Inertia body_rotate_inertia(const Quaternion& rot, const Inertia& inertia);

/**
 * @brief Transform mass properties
 *
 * The inertia tensor about the center of mass is rotated to the new frame \f$0\f$ from \f$a\f$:
 *
 * \f$ \mathbf{I}_0 = \mathbf{R}_{0a} \mathcal{I}_a \mathbf{R}^T_{0a} \f$
 *
 * The center of mass is transformed as
 *
 * \f$ \mathbf{p}_{0c} = \mathbf{R}_{0a} \mathbf{p}_{ac} + \mathbf{p}_{0a} \f$
 *
 * and the mass is constant.
 *
 * @param[in] transf Coordinate transform from old to new frame
 * @param[in] mass_props Mass properties
 * @return Mass properties in new coordinate frame
 */
MassProps body_transform_mass_props(const Transform& transf, const MassProps& mass_props);

/**
 * @brief Apply parallel axis theorem to inertia tensor
 *
 * The inertia tensor about the center of mass is translated to be about center of frame $0$
 * \f$ \mathcal{I}_0 = \mathcal{I} - m [\mathbf{p}_{0c}]^{\times} [\mathbf{p}_{0c}]^{\times} \f$
 *
 * @param[in] mass Mass of body
 * @param[in] com Center of mass
 * @param[in] inertia Inertia tensor with respect to center of mass
 * @return Output inertia tensor with respect to origin
 */
Inertia body_parallel_axis_inertia(real_t mass, const Vec3& com, const Inertia& inertia);

/**
 * @brief Combine mass properties of two bodies in origin of common frame
 *
 * Inertia is with respect to center of mass
 *
 * @param[in] mass_props_a Mass properties of body A
 * @param[in] transf_ab Coordinate transform from body A to body B
 * @param[in] mass_props_b Mass properties of body B
 * @return Combined mass properties
 */
MassProps body_combine(
    const MassProps& mass_props_a, const Transform& transf_ab, const MassProps& mass_props_b);

/**
 * @brief
 *
 * @param[in] inertia
 * @param[out] principal_inertia
 * @return true
 * @return false
 */
bool body_inertia_principal_axis(const Inertia& inertia, PrincipalInertia& principal_inertia);

/**
 * @brief Check if inertia tensor is positive definite
 *
 * @param inertia
 * @return true
 * @return false
 */
bool body_validate_inertia_is_pd(const Inertia& inertia);

/**
 * @}
 */

} // namespace fsb

#endif
