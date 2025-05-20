
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
using real_t = double;

/**
 * @brief 3D vector
 */
struct Vec3
{
    /** @brief X component */
    real_t x;
    /** @brief Y component */
    real_t y;
    /** @brief Z component */
    real_t z;
};

/**
 * @brief 3x3 symmetric matrix
 */
struct Mat3Sym
{
    /** @brief Element [0, 0] of matrix */
    real_t m00;
    /** @brief Element [1, 1] of matrix */
    real_t m11;
    /** @brief Element [2, 2] of matrix */
    real_t m22;
    /** @brief Element [0, 1] and [1, 0] of matrix */
    real_t m01;
    /** @brief Element [0, 2] and [2, 0] of matrix */
    real_t m02;
    /** @brief Element [1, 2] and [2, 1] of matrix */
    real_t m12;
};

/**
 * @brief 3x3 matrix, column-major
 */
struct Mat3
{
    /** @brief Element [0, 0] of matrix */
    real_t m00;
    /** @brief Element [1, 0] of matrix */
    real_t m10;
    /** @brief Element [2, 0] of matrix */
    real_t m20;
    /** @brief Element [0, 1] of matrix */
    real_t m01;
    /** @brief Element [1, 1] of matrix */
    real_t m11;
    /** @brief Element [2, 1] of matrix */
    real_t m21;
    /** @brief Element [0, 2] of matrix */
    real_t m02;
    /** @brief Element [1, 2] of matrix */
    real_t m12;
    /** @brief Element [2, 2] of matrix */
    real_t m22;
};

} // namespace fsb

#endif
