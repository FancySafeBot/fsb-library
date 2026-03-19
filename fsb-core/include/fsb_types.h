
#ifndef FSB_TYPES_H
#define FSB_TYPES_H

#include <array>
#include <cmath>
#include <cstddef>

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

/**
 * @brief Non-owning view over a contiguous sequence of elements (C++17 substitute for std::span)
 */
template<typename T>
class Span
{
public:
    Span() = default;
    Span(T* ptr, size_t len) noexcept : m_ptr(ptr), m_len(len) {}

    [[nodiscard]] T*       data()        noexcept { return m_ptr; }
    [[nodiscard]] const T* data()  const noexcept { return m_ptr; }
    [[nodiscard]] size_t   size()  const noexcept { return m_len; }
    [[nodiscard]] bool     empty() const noexcept { return m_len == 0U; }

    T&       operator[](size_t ind)       noexcept { return *(m_ptr + ind); }
    const T& operator[](size_t ind) const noexcept { return *(m_ptr + ind); }

    [[nodiscard]] T*       begin()       noexcept { return m_ptr; }
    [[nodiscard]] T*       end()         noexcept { return m_ptr + m_len; }
    [[nodiscard]] const T* begin() const noexcept { return m_ptr; }
    [[nodiscard]] const T* end()   const noexcept { return m_ptr + m_len; }

    [[nodiscard]] Span<T> active(size_t num, size_t off = 0U) noexcept
    {
        const size_t off_clamped = (off < m_len) ? off : m_len;
        const size_t remaining = m_len - off_clamped;
        const size_t count = (num < remaining) ? num : remaining;
        return Span<T>(m_ptr + off_clamped, count);
    }
    [[nodiscard]] Span<const T> active(size_t num, size_t off = 0U) const noexcept
    {
        const size_t off_clamped = (off < m_len) ? off : m_len;
        const size_t remaining = m_len - off_clamped;
        const size_t count = (num < remaining) ? num : remaining;
        return Span<const T>(m_ptr + off_clamped, count);
    }

private:
    T* m_ptr = nullptr;
    size_t m_len = 0U;
};

template<size_t MaxDim, typename Type = Real>
struct Array : public std::array<Type, MaxDim>
{
    Span<Type> active(size_t num) noexcept
    {
        const size_t num_count = (num < MaxDim) ? num : MaxDim;
        return Span<Type>(this->data(), num_count);
    }
    Span<const Type> active(size_t num) const noexcept
    {
        const size_t num_count = (num < MaxDim) ? num : MaxDim;
        return Span<const Type>(this->data(), num_count);
    }
};

} // namespace fsb

#endif
