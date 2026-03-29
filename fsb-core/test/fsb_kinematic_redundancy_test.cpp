#include <doctest/doctest.h>

#include <array>

#include "fsb_test_macros.h"

#include "fsb_body_tree.h"
#include "fsb_jacobian.h"
#include "fsb_joint.h"
#include "fsb_kinematic_redundancy.h"
#include <iostream>

TEST_SUITE_BEGIN("kinematic_redundancy");

static std::array<double, 36U> mat6_mul_jacobian_pinv(
    const fsb::Jacobian& jac, const fsb::Jacobian& jac_pinv, const size_t dofs)
{
    // (6 x dofs) * (dofs x 6) -> (6 x 6)
    std::array<double, 36U> out = {};
    for (size_t col = 0U; col < 6U; ++col)
    {
        for (size_t row = 0U; row < 6U; ++row)
        {
            double acc = 0.0;
            for (size_t k = 0U; k < dofs; ++k)
            {
                acc += jac.j[fsb::jacobian_index(row, k)]
                       * jac_pinv.j[fsb::joint_matrix_index(k, col, dofs)];
            }
            out[col * 6U + row] = acc;
        }
    }
    return out;
}

static fsb::Jacobian mat6_mul_jacobian(
    const std::array<double, 36U>& a_6x6, const fsb::Jacobian& jac, const size_t dofs)
{
    // (6 x 6) * (6 x dofs) -> (6 x dofs)
    fsb::Jacobian out = {};
    for (size_t col = 0U; col < dofs; ++col)
    {
        for (size_t row = 0U; row < 6U; ++row)
        {
            double acc = 0.0;
            for (size_t k = 0U; k < 6U; ++k)
            {
                acc += a_6x6[k * 6U + row] * jac.j[fsb::jacobian_index(k, col)];
            }
            out.j[fsb::jacobian_index(row, col)] = acc;
        }
    }
    return out;
}

static std::array<double, fsb::MaxSize::kDofs * fsb::MaxSize::kDofs> matn_mul_jacobian(
    const fsb::Jacobian& jac_pinv, const fsb::Jacobian& jac, const size_t dofs)
{
    // (dofs x 6) * (6 x dofs) -> (dofs x dofs)
    std::array<double, fsb::MaxSize::kDofs * fsb::MaxSize::kDofs> out = {};
    for (size_t col = 0U; col < dofs; ++col)
    {
        for (size_t row = 0U; row < dofs; ++row)
        {
            double acc = 0.0;
            for (size_t k = 0U; k < 6U; ++k)
            {
                acc += jac_pinv.j[fsb::joint_matrix_index(row, k, dofs)]
                       * jac.j[fsb::jacobian_index(k, col)];
            }
            out[col * dofs + row] = acc;
        }
    }
    return out;
}

static fsb::Jacobian matn_mul_jacobian_pinv(
    const std::array<double, fsb::MaxSize::kDofs * fsb::MaxSize::kDofs>& a_nxn,
    const fsb::Jacobian& jac_pinv,
    const size_t dofs)
{
    // (dofs x dofs) * (dofs x 6) -> (dofs x 6)
    fsb::Jacobian out = {};
    for (size_t col = 0U; col < 6U; ++col)
    {
        for (size_t row = 0U; row < dofs; ++row)
        {
            double acc = 0.0;
            for (size_t k = 0U; k < dofs; ++k)
            {
                acc += a_nxn[k * dofs + row] * jac_pinv.j[fsb::joint_matrix_index(k, col, dofs)];
            }
            out.j[fsb::joint_matrix_index(row, col, dofs)] = acc;
        }
    }
    return out;
}


static fsb::Jacobian make_identity_jacobian_6xn(size_t dofs)
{
    fsb::Jacobian jac = {};
    const size_t n = (dofs < fsb::MaxSize::kDofs) ? dofs : fsb::MaxSize::kDofs;

    // 6xN, column-major, with a 6x6 identity in the first 6 columns.
    for (size_t col = 0U; col < n; ++col)
    {
        for (size_t row = 0U; row < 6U; ++row)
        {
            jac.j[fsb::jacobian_index(row, col)] = (col < 6U && row == col) ? 1.0 : 0.0;
        }
    }
    return jac;
}

TEST_CASE("Pseudoinverse of Panda Jacobian" * doctest::description("[fsb_kinematic_redundancy][fsb::jacobian_pseudoinverse]"))
{
    constexpr size_t dofs = 7U;
    fsb::Jacobian jac = {{-0.32983697206672097, 0.17135057568505976, 0.0, 0.0, 0.0, 1.0, 0.24059937623889344, 0.374711327101604, -0.37012935286550924, -0.8414709848078963, 0.5403023058681395, 2.220446049250313e-16, -0.4309645894690529, 0.2383365536189267, -0.010703170917977886, -0.16996103804989837, -0.26469863354927736, 0.9492354180824409, -0.06537472209909931, -0.10037581974562429, 0.3922998357104994, 0.8797658890242653, -0.474742073981774, 0.025138490424573168, 0.11718056947053287, -0.06479866161461803, -0.001809611832704249, 0.4646023860415935, 0.8473588454165946, -0.25715289222311233, -0.06704004103038955, -0.12123783396091165, 0.00014323096740403995, 0.8750320460436324, -0.48387634323441875, -0.013513062376045326, -7.632783294297951e-17, 3.469446951953614e-17, 4.445228907190568e-18, 0.3168987059365215, 0.5515243494818595, 0.771619143168681}};

    const fsb::Jacobian jac_pinv_expected = {{-0.6070811859107488, 1.4490200143943484, -1.8610870001771112, 1.3060599598166356, -2.225967282363882, 0.07026108602566375, 2.2930924410921145, 0.27185082884437883, 2.7257510957626003, 1.217872094113728, 2.6103817040094, 1.1930966750770289, 0.15576633462335276, -1.535221203797208, -0.02257608699313691, -0.15036630178840693, -0.031809976614954986, 2.407306886519908, 0.008474568663924646, -2.556779471294719, -0.05198870279676335, 0.05684584658103792, 0.37755406592147034, 0.11847563950160754, 0.3615875461863522, 0.5343932372786446, 0.8951728415305188, -0.03742726201577534, 0.1265898214669738, -0.20054613682919548, 0.16817523368090545, -0.1798207514492677, 1.0019678172503175, -0.4969561145140537, -0.03986910460721366, 0.16695928792225725, -0.004397700825479125, -0.3805603282885986, -0.01884899962070688, -0.9366132883053869, -0.009547052604558846, 1.2360684313590902}};

    fsb::Jacobian jac_pinv = {};
    const auto err = fsb::jacobian_pseudoinverse(jac, jac_pinv, dofs);
    REQUIRE(err == fsb::LinalgErrorType::ERROR_NONE);

    const double eps = 1.0e-8;
    for (size_t col = 0U; col < 6U; ++col)
    {
        for (size_t row = 0U; row < dofs; ++row)
        {
            const double expected = jac_pinv_expected.j[fsb::joint_matrix_index(row, col, dofs)];
            const double actual = jac_pinv.j[fsb::joint_matrix_index(row, col, dofs)];
            REQUIRE(actual == FsbApprox(expected, eps));
        }
    }

}



TEST_CASE("Pseudoinverse of identity Jacobian" * doctest::description("[fsb_kinematic_redundancy][fsb::jacobian_pseudoinverse]"))
{
    constexpr size_t dofs = 6U;
    const fsb::Jacobian jac = make_identity_jacobian_6xn(dofs);

    fsb::Jacobian jac_pinv = {};
    const auto err = fsb::jacobian_pseudoinverse(jac, jac_pinv, dofs);
    REQUIRE(err == fsb::LinalgErrorType::ERROR_NONE);

    // jac_pinv is dofs x 6 (column-major) stored in a Jacobian-sized buffer.
    for (size_t col = 0U; col < 6U; ++col)
    {
        for (size_t row = 0U; row < dofs; ++row)
        {
            const double expected = (row == col) ? 1.0 : 0.0;
            REQUIRE(jac_pinv.j[fsb::joint_matrix_index(row, col, dofs)] == FsbApprox(expected));
        }
    }
}

TEST_CASE("Pseudoinverse satisfies Moore-Penrose identities" * doctest::description("[fsb_kinematic_redundancy][fsb::jacobian_pseudoinverse]"))
{
    // Use a deterministic 6x7 Jacobian with full row-rank (rank 6).
    constexpr size_t dofs = 7U;
    fsb::Jacobian jac = make_identity_jacobian_6xn(dofs);
    // Add a non-zero 7th column (doesn't change row-rank due to first 6 columns being I).
    jac.j[fsb::jacobian_index(0U, 6U)] = 1.0;
    jac.j[fsb::jacobian_index(1U, 6U)] = -2.0;
    jac.j[fsb::jacobian_index(2U, 6U)] = 0.5;
    jac.j[fsb::jacobian_index(3U, 6U)] = 3.0;
    jac.j[fsb::jacobian_index(4U, 6U)] = -1.0;
    jac.j[fsb::jacobian_index(5U, 6U)] = 0.25;

    fsb::Jacobian jac_pinv = {};
    const auto err = fsb::jacobian_pseudoinverse(jac, jac_pinv, dofs);
    REQUIRE(err == fsb::LinalgErrorType::ERROR_NONE);

    // Validate Moore–Penrose conditions:
    // 1) J * J+ * J == J
    // 2) J+ * J * J+ == J+
    // 3) (J * J+) is symmetric
    // 4) (J+ * J) is symmetric
    constexpr double eps = 1.0e-9;

    const auto jj_pinv = mat6_mul_jacobian_pinv(jac, jac_pinv, dofs); // 6x6
    const fsb::Jacobian jj_pinv_j = mat6_mul_jacobian(jj_pinv, jac, dofs); // 6xdofs
    for (size_t col = 0U; col < dofs; ++col)
    {
        for (size_t row = 0U; row < 6U; ++row)
        {
            const double expected = jac.j[fsb::jacobian_index(row, col)];
            const double actual = jj_pinv_j.j[fsb::jacobian_index(row, col)];
            REQUIRE(actual == FsbApprox(expected, eps));
        }
    }

    const auto pinv_j = matn_mul_jacobian(jac_pinv, jac, dofs); // dofs x dofs
    const fsb::Jacobian pinv_j_pinv = matn_mul_jacobian_pinv(pinv_j, jac_pinv, dofs); // dofs x 6
    for (size_t col = 0U; col < 6U; ++col)
    {
        for (size_t row = 0U; row < dofs; ++row)
        {
            const double expected = jac_pinv.j[fsb::joint_matrix_index(row, col, dofs)];
            const double actual = pinv_j_pinv.j[fsb::joint_matrix_index(row, col, dofs)];
            REQUIRE(actual == FsbApprox(expected, eps));
        }
    }

    for (size_t r = 0U; r < 6U; ++r)
    {
        for (size_t c = 0U; c < 6U; ++c)
        {
            REQUIRE(jj_pinv[c * 6U + r] == FsbApprox(jj_pinv[r * 6U + c], eps));
        }
    }

    for (size_t r = 0U; r < dofs; ++r)
    {
        for (size_t c = 0U; c < dofs; ++c)
        {
            REQUIRE(pinv_j[c * dofs + r] == FsbApprox(pinv_j[r * dofs + c], eps));
        }
    }
}

TEST_CASE("Pseudoinverse works for rank-deficient Jacobian" * doctest::description("[fsb_kinematic_redundancy][fsb::jacobian_pseudoinverse]"))
{
    // 6x7 with rank 5: first 5 columns identity, remaining columns zero.
    constexpr size_t dofs = 7U;
    fsb::Jacobian jac = {};
    for (size_t col = 0U; col < dofs; ++col)
    {
        for (size_t row = 0U; row < 6U; ++row)
        {
            jac.j[fsb::jacobian_index(row, col)] = (col < 5U && row == col) ? 1.0 : 0.0;
        }
    }

    fsb::Jacobian jac_pinv = {};
    const auto err = fsb::jacobian_pseudoinverse(jac, jac_pinv, dofs);
    REQUIRE(err == fsb::LinalgErrorType::ERROR_NONE);

    // Check J * J+ * J == J (most important identity for rank-deficient case)
    constexpr double eps = 1.0e-8;
    const auto jj_pinv = mat6_mul_jacobian_pinv(jac, jac_pinv, dofs);
    const fsb::Jacobian jj_pinv_j = mat6_mul_jacobian(jj_pinv, jac, dofs);
    for (size_t col = 0U; col < dofs; ++col)
    {
        for (size_t row = 0U; row < 6U; ++row)
        {
            const double expected = jac.j[fsb::jacobian_index(row, col)];
            const double actual = jj_pinv_j.j[fsb::jacobian_index(row, col)];
            REQUIRE(actual == FsbApprox(expected, eps));
        }
    }
}

TEST_CASE("Nullspace motion is zero for 6x6 identity Jacobian" * doctest::description("[fsb_kinematic_redundancy][fsb::compute_nullspace_motion]"))
{
    constexpr size_t dofs = 6U;
    const fsb::Jacobian jac = make_identity_jacobian_6xn(dofs);

    fsb::JointSpace qd = {};
    qd = {0.2, -0.1, 0.05, 0.33, -0.7, 1.25};

    // Overload without explicit pseudoinverse
    {
        const fsb::JointSpace qn = fsb::compute_nullspace_motion(jac, qd, dofs);
        for (size_t i = 0U; i < dofs; ++i)
        {
            REQUIRE(qn[i] == FsbApprox(0.0));
        }
    }

    // Overload with explicit pseudoinverse
    {
        fsb::Jacobian jac_pinv = {};
        const auto err = fsb::jacobian_pseudoinverse(jac, jac_pinv, dofs);
        REQUIRE(err == fsb::LinalgErrorType::ERROR_NONE);

        const fsb::JointSpace qn = fsb::compute_nullspace_motion(jac, jac_pinv, qd, dofs);
        for (size_t i = 0U; i < dofs; ++i)
        {
            REQUIRE(qn[i] == FsbApprox(0.0));
        }
    }
}

TEST_CASE("Nullspace projection preserves pure nullspace component" * doctest::description("[fsb_kinematic_redundancy][fsb::compute_nullspace_motion]"))
{
    // 6x7 Jacobian: first 6 columns form identity; 7th column is zero.
    constexpr size_t dofs = 7U;
    const fsb::Jacobian jac = make_identity_jacobian_6xn(dofs);

    fsb::JointSpace qd = {};
    qd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4};

    // Without explicit pseudoinverse
    {
        const fsb::JointSpace qn = fsb::compute_nullspace_motion(jac, qd, dofs);
        for (size_t i = 0U; i < 6U; ++i)
        {
            REQUIRE(qn[i] == FsbApprox(0.0));
        }
        REQUIRE(qn[6] == FsbApprox(0.4));
    }

    // With explicit pseudoinverse
    {
        fsb::Jacobian jac_pinv = {};
        const auto err = fsb::jacobian_pseudoinverse(jac, jac_pinv, dofs);
        REQUIRE(err == fsb::LinalgErrorType::ERROR_NONE);

        const fsb::JointSpace qn = fsb::compute_nullspace_motion(jac, jac_pinv, qd, dofs);
        for (size_t i = 0U; i < 6U; ++i)
        {
            REQUIRE(qn[i] == FsbApprox(0.0));
        }
        REQUIRE(qn[6] == FsbApprox(0.4));
    }
}

TEST_CASE("Joint limit avoidance pushes away from limits and clamps" * doctest::description("[fsb_kinematic_redundancy][fsb::joint_limit_avoidance_objective]"))
{
    constexpr size_t dofs = 3U;

    fsb::JointSpacePosition q = {};
    q[0] = -0.9;
    q[1] = 0.0;
    q[2] = 0.9;

    fsb::JointLimits limits = {};
    for (size_t i = 0U; i < dofs; ++i)
    {
        limits.set[i] = true;
        limits.lower_position[i] = -1.0;
        limits.upper_position[i] = 1.0;
        limits.max_velocity[i] = 10.0;
    }

    // Disable middle joint; it should stay zero.
    limits.set[1] = false;

    const fsb::JointSpace qd = fsb::joint_limit_avoidance_objective(q, limits, dofs, 0.1);

    REQUIRE(qd[0] > 0.0);
    REQUIRE(qd[1] == FsbApprox(0.0));
    REQUIRE(qd[2] < 0.0);

    // Midpoint with symmetric limits produces ~0.
    q[0] = 0.0;
    limits.set[0] = true;
    const fsb::JointSpace qd_mid = fsb::joint_limit_avoidance_objective(q, limits, dofs, 0.1);
    REQUIRE(qd_mid[0] == FsbApprox(0.0));

    // Clamp check: make it saturate.
    q[0] = -0.999;
    limits.max_velocity[0] = 0.05;
    const fsb::JointSpace qd_clamped = fsb::joint_limit_avoidance_objective(q, limits, dofs, 100.0);
    REQUIRE(qd_clamped[0] == FsbApprox(0.05));
}

TEST_SUITE_END();
