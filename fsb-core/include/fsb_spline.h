#ifndef FSB_SPLINE_H
#define FSB_SPLINE_H

#include <array>
#include "fsb_trajectory_types.h"

namespace fsb
{

using SplineCoeffs = std::array<Real, 4U>;

template <size_t MaxPoints>
class Spline final : public SegmentScalar
{
public:
    Spline() = default;

    template <size_t NumPoints>
    bool generate(const Real step_size, const std::array<Real, NumPoints>& points)
    {
        static_assert(NumPoints <= MaxPoints, "NumPoints must be <= MaxPoints");
        if constexpr (NumPoints < 2U)
        {
            return false;
        }
        if (step_size <= FSB_TOL)
        {
            return false;
        }

        m_num_points = NumPoints;
        for (size_t i = 0U; i < NumPoints; ++i)
        {
            m_points[i] = points[i];
        }

        m_start_time = 0.0;
        m_step_size = step_size;
        m_duration = static_cast<Real>(m_num_points - 1) * m_step_size;

        return build_natural_spline();
    }

    template <size_t NumPoints>
    bool append(const std::array<Real, NumPoints>& points)
    {
        static_assert(NumPoints <= MaxPoints, "NumPoints must be <= MaxPoints");

        if constexpr (NumPoints == 0U)
        {
            return true;
        }

        if (m_num_points == 0U)
        {
            return false;
        }

        if ((m_num_points + NumPoints) > MaxPoints)
        {
            return false;
        }

        for (size_t i = 0U; i < NumPoints; ++i)
        {
            m_points[m_num_points + i] = points[i];
        }
        m_num_points += NumPoints;
        m_duration = static_cast<Real>(m_num_points - 1U) * m_step_size;

        return build_natural_spline();
    }

    [[nodiscard]] TrajState evaluate(Real t_eval) const override final
    {
        if (m_num_points < 2U)
        {
            return {};
        }

        Real t_local = t_eval - m_start_time;
        if (t_local < 0.0)
        {
            t_local = 0.0;
        }
        if (t_local > m_duration)
        {
            t_local = m_duration;
        }

        const Real t_index = t_local / m_step_size;
        size_t k = static_cast<size_t>(t_index);
        const size_t max_segment = m_num_points - 2U;
        if (k > max_segment)
        {
            k = max_segment;
        }

        const Real dt = t_index - static_cast<Real>(k);
        const Real a = m_spline[k][0];
        const Real b = m_spline[k][1];
        const Real c = m_spline[k][2];
        const Real d = m_spline[k][3];

        const Real inv_h = 1.0 / m_step_size;
        const Real inv_h2 = inv_h * inv_h;
        const Real inv_h3 = inv_h2 * inv_h;

        const Real position = (((a * dt) + b) * dt + c) * dt + d;
        const Real velocity = (((3.0 * a * dt) + (2.0 * b)) * dt + c) * inv_h;
        const Real acceleration = ((6.0 * a * dt) + (2.0 * b)) * inv_h2;
        const Real jerk = (6.0 * a) * inv_h3;

        return {position, velocity, acceleration, jerk};
    }

    [[nodiscard]] TrajState get_final_state() const override final
    {
        return evaluate(get_final_time());
    }

    [[nodiscard]] TrajState get_initial_state() const override final
    {
        return evaluate(m_start_time);
    }

    [[nodiscard]] Real get_start_time() const override final
    {
        return m_start_time;
    }

    [[nodiscard]] Real get_duration() const override final
    {
        return m_duration;
    }

    [[nodiscard]] Real get_final_time() const override final
    {
        return m_start_time + m_duration;
    }

private:
    /**
     * @brief Build a natural cubic spline using a tridiagonal solve (Thomas method).
     *
     * Derivation follows the standard natural cubic spline formulation (e.g., de Boor,
     * "A Practical Guide to Splines"): second derivatives are solved at knots with
     * natural endpoint conditions. This implementation assumes uniformly spaced knots
     * (`step_size`) and natural endpoints. Here `Mi` denotes the spline second
     * derivative at knot `i` (`Mi = S''(ti)`), so `M0` is at the first knot and
     * `Mn` is at the last knot (`n = m_num_points - 1`), with `M0 = Mn = 0`.
     */
    [[nodiscard]] bool build_natural_spline()
    {
        if (m_num_points < 2U)
        {
            return false;
        }

        for (size_t i = 0U; i < MaxPoints; ++i)
        {
            m_second_derivative[i] = 0.0;
        }

        if (m_num_points > 2U)
        {
            std::array<Real, MaxPoints> c_prime = {};
            std::array<Real, MaxPoints> d_prime = {};

            c_prime[1] = 1.0 / 4.0;
            d_prime[1] = 6.0 * (m_points[2] - (2.0 * m_points[1]) + m_points[0]) / 4.0;

            for (size_t i = 2U; i < (m_num_points - 1U); ++i)
            {
                const Real rhs = 6.0 * (m_points[i + 1U] - (2.0 * m_points[i]) + m_points[i - 1U]);
                const Real denom = 4.0 - c_prime[i - 1U];
                c_prime[i] = 1.0 / denom;
                d_prime[i] = (rhs - d_prime[i - 1U]) / denom;
            }

            m_second_derivative[m_num_points - 2U] = d_prime[m_num_points - 2U];
            for (size_t i = m_num_points - 2U; i > 1U; --i)
            {
                m_second_derivative[i - 1U] = d_prime[i - 1U] - (c_prime[i - 1U] * m_second_derivative[i]);
            }
        }

        for (size_t k = 0U; k < (m_num_points - 1U); ++k)
        {
            const Real m0 = m_second_derivative[k];
            const Real m1 = m_second_derivative[k + 1U];
            const Real y0 = m_points[k];
            const Real y1 = m_points[k + 1U];

            m_spline[k][0] = (m1 - m0) / 6.0;
            m_spline[k][1] = m0 / 2.0;
            m_spline[k][2] = (y1 - y0) - ((2.0 * m0 + m1) / 6.0);
            m_spline[k][3] = y0;
        }

        return true;
    }

    Real m_start_time = 0.0;
    Real m_step_size = 0.0;
    Real m_duration = 0.0;

    std::array<Real, MaxPoints> m_points = {};
    std::array<Real, MaxPoints> m_second_derivative = {};
    std::array<SplineCoeffs, MaxPoints> m_spline = {};
    size_t m_num_points = 0U;
};


}

#endif // FSB_SPLINE_H
