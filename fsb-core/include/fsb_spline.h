#ifndef FSB_SPLINE_H
#define FSB_SPLINE_H

#include <array>
#include <cstdint>
#include "fsb_trajectory_types.h"

namespace fsb
{

using SplineCoeffs = std::array<Real, 4U>;

enum class SplineEndCondition : uint8_t
{
    Natural = 0U,
    Clamped = 1U
};

enum class SplineError : uint8_t
{
    SUCCESS = 0U,
    INVALID_STEP_SIZE = 1U,
    NOT_ENOUGH_POINTS = 2U,
    TOO_MANY_POINTS = 3U,
    NOT_GENERATED = 4U,
    SINGULAR_SYSTEM = 5U
};

template <size_t MaxPoints>
class Spline final : public SegmentScalar
{
public:
    Spline() = default;

    template <size_t NumPoints>
    [[nodiscard]] SplineError generate(const Real step_size,
        const std::array<Real, NumPoints>& points,
        const SplineEndCondition end_condition = SplineEndCondition::Natural,
        const Real start_slope = 0.0,
        const Real end_slope = 0.0)
    {
        if (NumPoints > MaxPoints)
        {
            return SplineError::TOO_MANY_POINTS;
        }
        if (NumPoints < 2U)
        {
            return SplineError::NOT_ENOUGH_POINTS;
        }
        if (step_size <= FSB_TOL)
        {
            return SplineError::INVALID_STEP_SIZE;
        }

        const SplineBuildParameters parameters = {NumPoints, step_size, end_condition,
            start_slope, end_slope};
        const SplineBuildResult result = build_spline(points, parameters);
        if (result.error == SplineError::SUCCESS)
        {
            m_start_time = 0.0;
            m_step_size = step_size;
            m_duration = static_cast<Real>(NumPoints - 1U) * step_size;
            m_end_condition = end_condition;
            m_start_slope = start_slope;
            m_end_slope = end_slope;
            for (size_t i = 0U; i < NumPoints; ++i)
            {
                m_points[i] = points[i];
            }
            m_num_points = NumPoints;
            m_spline = result.spline;
        }

        return result.error;
    }

    [[nodiscard]] TrajState evaluate(Real t_eval) const override
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
        const auto k = static_cast<size_t>(t_index);
        const size_t max_segment = m_num_points - 2U;
        const auto k_clamped = (k > max_segment) ? max_segment : k;

        const Real dt = t_index - static_cast<Real>(k_clamped);
        const Real a = m_spline[k_clamped][0];
        const Real b = m_spline[k_clamped][1];
        const Real c = m_spline[k_clamped][2];
        const Real d = m_spline[k_clamped][3];

        const Real inv_h = 1.0 / m_step_size;
        const Real inv_h2 = inv_h * inv_h;
        const Real inv_h3 = inv_h2 * inv_h;

        const Real position = (((a * dt) + b) * dt + c) * dt + d;
        const Real velocity = (((3.0 * a * dt) + (2.0 * b)) * dt + c) * inv_h;
        const Real acceleration = ((6.0 * a * dt) + (2.0 * b)) * inv_h2;
        const Real jerk = (6.0 * a) * inv_h3;

        return {position, velocity, acceleration, jerk};
    }

    [[nodiscard]] TrajState get_final_state() const override
    {
        return evaluate(get_final_time());
    }

    [[nodiscard]] TrajState get_initial_state() const override
    {
        return evaluate(m_start_time);
    }

    [[nodiscard]] Real get_start_time() const override
    {
        return m_start_time;
    }

    [[nodiscard]] Real get_duration() const override
    {
        return m_duration;
    }

    [[nodiscard]] Real get_final_time() const override
    {
        return m_start_time + m_duration;
    }

private:
    struct SplineBuildParameters
    {
        size_t num_points = 0U;
        Real step_size = 0.0;
        SplineEndCondition end_condition = SplineEndCondition::Natural;
        Real start_slope = 0.0;
        Real end_slope = 0.0;
    };

    struct SplineBuildResult
    {
        SplineError error = SplineError::SUCCESS;
        std::array<Real, MaxPoints> second_derivative = {};
        std::array<SplineCoeffs, MaxPoints> spline = {};
    };

    template <size_t NumPoints>
    [[nodiscard]] static SplineBuildResult build_spline(
        const std::array<Real, NumPoints>& points, const SplineBuildParameters& parameters)
    {
        SplineBuildResult result = {};

        if (parameters.end_condition == SplineEndCondition::Natural)
        {
            result = build_natural_spline(points, parameters);
        }
        else
        {
            result = build_clamped_spline(points, parameters);
        }

        return result;
    }

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
    template <size_t NumPoints>
    [[nodiscard]] static SplineBuildResult build_natural_spline(
        const std::array<Real, NumPoints>& points, const SplineBuildParameters& parameters)
    {
        SplineBuildResult result = {};

        if (parameters.num_points < 2U)
        {
            result.error = SplineError::NOT_ENOUGH_POINTS;
            return result;
        }

        if (parameters.num_points > 2U)
        {
            std::array<Real, MaxPoints> c_prime = {};
            std::array<Real, MaxPoints> d_prime = {};

            c_prime[1] = 1.0 / 4.0;
            d_prime[1] = 6.0 * (points[2] - (2.0 * points[1]) + points[0]) / 4.0;

            for (size_t i = 2U; i < (parameters.num_points - 1U); ++i)
            {
                const Real rhs = 6.0 * (points[i + 1U] - (2.0 * points[i]) + points[i - 1U]);
                const Real denom = 4.0 - c_prime[i - 1U];
                c_prime[i] = 1.0 / denom;
                d_prime[i] = (rhs - d_prime[i - 1U]) / denom;
            }

            result.second_derivative[parameters.num_points - 2U] = d_prime[parameters.num_points - 2U];
            for (size_t i = parameters.num_points - 2U; i > 1U; --i)
            {
                result.second_derivative[i - 1U] = d_prime[i - 1U]
                    - (c_prime[i - 1U] * result.second_derivative[i]);
            }
        }

        for (size_t k = 0U; k < (parameters.num_points - 1U); ++k)
        {
            const Real m0 = result.second_derivative[k];
            const Real m1 = result.second_derivative[k + 1U];
            const Real y0 = points[k];
            const Real y1 = points[k + 1U];

            result.spline[k][0] = (m1 - m0) / 6.0;
            result.spline[k][1] = m0 / 2.0;
            result.spline[k][2] = (y1 - y0) - ((2.0 * m0 + m1) / 6.0);
            result.spline[k][3] = y0;
        }

        return result;
    }

    /**
     * @brief Build a clamped cubic spline with specified endpoint slopes.
     *
     * Endpoint conditions are S'(t0) = start_slope and S'(tn) = end_slope,
     * where slopes are in value-per-second units.
     */
    template <size_t NumPoints>
    [[nodiscard]] static SplineBuildResult build_clamped_spline(
        const std::array<Real, NumPoints>& points, const SplineBuildParameters& parameters)
    {
        SplineBuildResult result = {};

        if (parameters.num_points < 2U)
        {
            result.error = SplineError::NOT_ENOUGH_POINTS;
            return result;
        }

        const size_t n = parameters.num_points - 1U;

        std::array<Real, MaxPoints> lower = {};
        std::array<Real, MaxPoints> diag = {};
        std::array<Real, MaxPoints> upper = {};
        std::array<Real, MaxPoints> rhs = {};

        const Real start_slope_scaled = parameters.start_slope * parameters.step_size;
        const Real end_slope_scaled = parameters.end_slope * parameters.step_size;

        diag[0] = 2.0;
        upper[0] = 1.0;
        rhs[0] = 6.0 * ((points[1] - points[0]) - start_slope_scaled);

        for (size_t i = 1U; i < n; ++i)
        {
            lower[i] = 1.0;
            diag[i] = 4.0;
            upper[i] = 1.0;
            rhs[i] = 6.0 * (points[i + 1U] - (2.0 * points[i]) + points[i - 1U]);
        }

        lower[n] = 1.0;
        diag[n] = 2.0;
        rhs[n] = 6.0 * (end_slope_scaled - (points[n] - points[n - 1U]));

        for (size_t i = 1U; i <= n; ++i)
        {
            if ((diag[i - 1U] > -FSB_TOL) && (diag[i - 1U] < FSB_TOL))
            {
                result.error = SplineError::SINGULAR_SYSTEM;
                return result;
            }

            const Real w = lower[i] / diag[i - 1U];
            diag[i] -= w * upper[i - 1U];
            rhs[i] -= w * rhs[i - 1U];
        }

        if ((diag[n] > -FSB_TOL) && (diag[n] < FSB_TOL))
        {
            result.error = SplineError::SINGULAR_SYSTEM;
            return result;
        }
        result.second_derivative[n] = rhs[n] / diag[n];

        for (size_t i = n; i > 0U; --i)
        {
            if ((diag[i - 1U] > -FSB_TOL) && (diag[i - 1U] < FSB_TOL))
            {
                result.error = SplineError::SINGULAR_SYSTEM;
                return result;
            }
            result.second_derivative[i - 1U] =
                (rhs[i - 1U] - (upper[i - 1U] * result.second_derivative[i])) / diag[i - 1U];
        }

        for (size_t k = 0U; k < n; ++k)
        {
            const Real m0 = result.second_derivative[k];
            const Real m1 = result.second_derivative[k + 1U];
            const Real y0 = points[k];
            const Real y1 = points[k + 1U];

            result.spline[k][0] = (m1 - m0) / 6.0;
            result.spline[k][1] = m0 / 2.0;
            result.spline[k][2] = (y1 - y0) - ((2.0 * m0 + m1) / 6.0);
            result.spline[k][3] = y0;
        }

        return result;
    }

private:
    Real m_start_time = 0.0;
    Real m_step_size = 0.0;
    Real m_duration = 0.0;
    SplineEndCondition m_end_condition = SplineEndCondition::Natural;
    Real m_start_slope = 0.0;
    Real m_end_slope = 0.0;

    std::array<Real, MaxPoints> m_points = {};
    std::array<SplineCoeffs, MaxPoints> m_spline = {};
    size_t m_num_points = 0U;
};

}

#endif // FSB_SPLINE_H
