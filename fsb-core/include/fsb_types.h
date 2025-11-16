
#ifndef FSB_TYPES_H
#define FSB_TYPES_H

/**
 * @brief Floating point tolerance
 */
#define FSB_TOL (4.5e-15)

/**
 * @brief Cartesian vector size
 */
#define FSB_CART_SIZE (6U)

namespace fsb
{

/**
 * @brief Real number type
 */
using Real = double;

/**
 * @brief 3D vector
 */
struct Vec3
{
    /** @brief X component */
    Real x = 0.0;
    /** @brief Y component */
    Real y = 0.0;
    /** @brief Z component */
    Real z = 0.0;
};

/**
 * @brief 3x3 symmetric matrix
 */
struct Mat3Sym
{
    /** @brief Element [0, 0] of matrix */
    Real m00 = 0.0;
    /** @brief Element [1, 1] of matrix */
    Real m11 = 0.0;
    /** @brief Element [2, 2] of matrix */
    Real m22 = 0.0;
    /** @brief Element [0, 1] and [1, 0] of matrix */
    Real m01 = 0.0;
    /** @brief Element [0, 2] and [2, 0] of matrix */
    Real m02 = 0.0;
    /** @brief Element [1, 2] and [2, 1] of matrix */
    Real m12 = 0.0;
};

/**
 * @brief 3x3 matrix
 */
struct Mat3
{
    /** @brief Element [0, 0] of matrix */
    Real m00 = 0.0;
    /** @brief Element [1, 0] of matrix */
    Real m10 = 0.0;
    /** @brief Element [2, 0] of matrix */
    Real m20 = 0.0;
    /** @brief Element [0, 1] of matrix */
    Real m01 = 0.0;
    /** @brief Element [1, 1] of matrix */
    Real m11 = 0.0;
    /** @brief Element [2, 1] of matrix */
    Real m21 = 0.0;
    /** @brief Element [0, 2] of matrix */
    Real m02 = 0.0;
    /** @brief Element [1, 2] of matrix */
    Real m12 = 0.0;
    /** @brief Element [2, 2] of matrix */
    Real m22 = 0.0;
};

} // namespace fsb

#endif
