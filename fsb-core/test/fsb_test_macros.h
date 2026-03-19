#pragma once

#include <cmath>
#include <limits>
#include <doctest/doctest.h>
#include "fsb_types.h"

/**
 * @brief Custom floating point comparison for unit tests
 *
 * \f$ |x - \hat{x}| < \epsilon \left( s + \max(|x|, |\hat{x}|) \right) \f$
 */
struct FsbApprox : doctest::Approx
{
    constexpr static fsb::Real default_epsilon = 1.0e3 * std::numeric_limits<fsb::Real>::epsilon();

    explicit FsbApprox(fsb::Real value, fsb::Real eps = default_epsilon) : doctest::Approx(value)
    {
        epsilon(eps);
        scale((fabs(value) < eps) ? 0.5 : 0.0);
    }
};

inline doctest::String toString(const FsbApprox& in)
{
    return doctest::toString(in.m_value);
}
