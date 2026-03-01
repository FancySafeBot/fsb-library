#include <doctest/doctest.h>
#include <array>
#include "fsb_test_macros.h"
#include "fsb_linalg.h"

TEST_SUITE_BEGIN("linalg");

TEST_CASE("Singular Value Decomposition" * doctest::description("[fsb_linalg][fsb_linalg_svd]"))
{
    // Inputs
    constexpr size_t rows = 3U;
    constexpr size_t cols = 4U;
    const std::array<double_t, rows * cols> a_mat = {
        0.957166948242946, 0.485375648722841, 0.800280468888800,
        0.141886338627215, 0.421761282626275, 0.915735525189067,
        0.792207329559554, 0.959492426392903, 0.655740699156587,
        0.0357116785741896, 0.849129305868777, 0.933993247757551
    };
    constexpr size_t work_len = 256U;
    std::array<double_t, work_len> work = {};
    const bool u_full = false;
    const bool v_full = false;

    // Expected
    const std::array<double_t, rows * rows> u_expected = {
        -0.436686793704996, -0.585976373201268, -0.682595293166851,
        0.891072308003706, -0.177432382809533, -0.417741416954892,
        0.123672091082020, -0.790663923282465, 0.599629697652627
    };
    const std::array<double_t, FSB_MIN(rows, cols)> s_expected = {
        2.36519644590068, 0.790034423136396, 0.428179268650398
    };
    const std::array<double_t, FSB_MIN(rows, cols) * cols> vt_expected = {
        -0.527934375763356, 0.547410712881673, 0.500906453486051, -0.394968797595884,
        -0.418897921755130, 0.544580700083686, -0.573225910244715, 0.331300674476610,
        -0.624646104104312, -0.486515158692034, -0.644287129828739, -0.249683460765213
    };
    FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_ERROR_NONE;

    // Actual
    std::array<double_t, rows * rows> u_actual = {};
    std::array<double_t, FSB_MIN(rows, cols)> s_actual = {};
    std::array<double_t, cols * cols> vt_actual = {};
    // Process
    const FsbLinalgErrorType err_actual = fsb_linalg_svd_array<rows, cols, work_len>(
        a_mat, u_full, v_full, work, u_actual, s_actual, vt_actual);

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < (rows * rows); ++k)
    {
        REQUIRE(u_actual[k] == FsbApprox(u_expected[k]));
    }
    for (size_t k = 0U; k < rows; ++k)
    {
        REQUIRE(s_actual[k] == FsbApprox(s_expected[k]));
    }
    for (size_t k = 0U; k < (FSB_MIN(rows, cols) * cols); ++k)
    {
        REQUIRE(vt_actual[k] == FsbApprox(vt_expected[k]));
    }
}

TEST_CASE("Eigenvalue decomposition" * doctest::description("[fsb_linalg][fsb_linalg_matrix_eig]"))
{
    // Inputs
    constexpr size_t adim = 3U;
    const std::array<double_t, adim * adim> a_mat = {
        0.792207329559554, 0.959492426392903, 0.655740699156587,
        0.0357116785741896, 0.849129305868777, 0.933993247757551,
        0.678735154857774, 0.757740130578333, 0.743132468124916
    };
    constexpr size_t work_len = 400U;
    std::array<double_t, work_len> work = {};

    // Expected
    const std::array<double_t, adim> eig_val_real_expected = {
        2.08316648761762, 0.150651307967817, 0.150651307967817
    };
    const std::array<double_t, adim> eig_val_imag_expected = {
       0.0, 0.234508337575288, -0.234508337575288
    };
    const std::array<double_t, adim * adim> eig_vec_real_expected = {
        0.358017325865748, 0.674696102475708, 0.645452371352083,
        0.616826825411530, -0.00999200711587476, -0.666934264156766,
        0.616826825411530, -0.00999200711587476, -0.666934264156766
    };
    const std::array<double_t, adim * adim> eig_vec_imag_expected = {
        0.0, 0.0, 0.0,
        0.244339279699821, -0.339001225567072, 0.0,
        -0.244339279699821, 0.339001225567072, 0.0
    };
    FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_ERROR_NONE;
    // Actual
    std::array<double_t, adim> eig_val_real_actual = {};
    std::array<double_t, adim> eig_val_imag_actual = {};
    std::array<double_t, adim * adim> eig_vec_real_actual = {};
    std::array<double_t, adim * adim> eig_vec_imag_actual = {};
    // Process
    const FsbLinalgErrorType err_actual = fsb_linalg_matrix_eig_array<adim, work_len>(
        a_mat, work,
        eig_val_real_actual, eig_val_imag_actual,
        eig_vec_real_actual, eig_vec_imag_actual);

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < adim; ++k)
    {
        REQUIRE(eig_val_real_actual[k] == FsbApprox(eig_val_real_expected[k]));
    }
    for (size_t k = 0U; k < adim; ++k)
    {
        REQUIRE(eig_val_imag_actual[k] == FsbApprox(eig_val_imag_expected[k]));
    }
    for (size_t k = 0U; k < (adim * adim); ++k)
    {
        REQUIRE(eig_vec_real_actual[k] == FsbApprox(eig_vec_real_expected[k]));
    }
    for (size_t k = 0U; k < (adim * adim); ++k)
    {
        REQUIRE(eig_vec_imag_actual[k] == FsbApprox(eig_vec_imag_expected[k]));
    }
}

TEST_CASE("Eigenvalue decomposition of symmetric positive definite" * doctest::description("[fsb_linalg][fsb_linalg_sym_lt_eig]"))
{
    // Inputs
    constexpr size_t adim = 3U;
    const std::array<double_t, adim * adim> a_mat = {
        5745.0, -55.722221, 136.4,
        0.0, 7073.0, 49.635477,
        0.0, 0.0, 2614.0
    };
    constexpr size_t work_len = 128U;
    std::array<double_t, work_len> work = {};

    // Expected
    const std::array<double_t, adim> eig_val_expected = {
        2607.46319128460, 5748.76982107806, 7075.76698763734
    };
    const std::array<double_t, adim * adim> eig_vec_expected = {
        -0.0436361061779135, -0.0116483932490119, 0.998979581959685,
        0.998213049251589, 0.0403517891385511, 0.0440731371377369,
        0.0408239946780535, -0.999117634739791, -0.00986678318865923
    };
    FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_ERROR_NONE;
    // Actual
    std::array<double_t, adim> eig_val_actual = {};
    std::array<double_t, adim * adim> eig_vec_actual = {};
    // Process
    const FsbLinalgErrorType err_actual = fsb_linalg_sym_lt_eig_array<adim, work_len>(
        a_mat, work, eig_val_actual, eig_vec_actual);

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < adim; ++k)
    {
        REQUIRE(eig_val_actual[k] == FsbApprox(eig_val_expected[k]));
    }
    for (size_t k = 0U; k < (adim * adim); ++k)
    {
        if (eig_vec_actual[k] * eig_vec_expected[k] < 0.0)
        {
            REQUIRE(-eig_vec_actual[k] == FsbApprox(eig_vec_expected[k]));
        }
        else
        {
            REQUIRE(eig_vec_actual[k] == FsbApprox(eig_vec_expected[k]));
        }
    }
}

TEST_CASE("Cholesky factorization of not positive definite matrix" * doctest::description("[fsb_linalg][fsb_linalg_sym_lt_chol]"))
{
    // Inputs
    constexpr size_t adim = 3U;
    const std::array<double_t, adim * adim> a_mat = {
        0.814723686393179, 0.905791937075619, 0.126986816293506,
        0.0, 0.632359246225410, 0.0975404049994095,
        0.0, 0.0, 0.957506835434298
    };
    // Expected
    FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_NOT_POSITIVE_DEFINITE;
    // Actual
    std::array<double_t, adim * adim> chol_actual = {};
    // Process
    const FsbLinalgErrorType err_actual = fsb_linalg_cholesky_decomposition_array<adim>(
        a_mat, chol_actual);

    // Check result
    REQUIRE(err_actual == err_expected);
}

TEST_CASE("Cholesky factorization of symmetric positive definite matrix" * doctest::description("[fsb_linalg][fsb_linalg_sym_lt_chol]"))
{
    // Inputs
    constexpr size_t adim = 3U;
    const std::array<double_t, adim * adim> a_mat = {
        1.814723686393179, 0.905791937075619, 0.126986816293506,
        0.0, 1.632359246225410, 0.0975404049994095,
        0.0, 0.0, 1.957506835434298
    };
    // Expected
    const std::array<double_t, adim * adim> chol_expected = {
        1.34711680502961, 0.672393020184847, 0.0942656314726287,
        0.0, 1.08639167551676, 0.0314406425635117,
        0.0, 0.0, 1.39557597863841
    };
    FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_ERROR_NONE;
    // Actual
    std::array<double_t, adim * adim> chol_actual = {};
    // Process
    const FsbLinalgErrorType err_actual = fsb_linalg_cholesky_decomposition_array<adim>(
        a_mat, chol_actual);

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < (adim * adim); ++k)
    {
        REQUIRE(chol_actual[k] == FsbApprox(chol_expected[k]));
    }
}


TEST_CASE("Cholesky solve not positive definite matrix" * doctest::description("[fsb_linalg][fsb_linalg_sym_lt_chol]"))
{
    // Inputs
    constexpr size_t adim = 3U;
    constexpr size_t nrhs = 2U;
    const std::array<double_t, adim * adim> a_mat = {
        -4.0, 12.0, -16.0,
        0.0, 37.0, -43.0,
        0.0, 0.0, 98.0
    };
    const std::array<double_t, nrhs * adim> b_vec = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    constexpr size_t work_len = adim * adim;
    std::array<double_t, work_len> work = {};
    // Expected
    const FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_NOT_POSITIVE_DEFINITE;
    const std::array<double_t, nrhs * adim> x_vec_expected = {};
    // Process
    std::array<double_t, nrhs * adim> x_vec_actual = {};
    const FsbLinalgErrorType err_actual = fsb_linalg_cholesky_solve_array<adim, nrhs, work_len>(
        a_mat, b_vec, work, x_vec_actual);

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < (nrhs * adim); ++k)
    {
        REQUIRE(x_vec_actual[k] == FsbApprox(x_vec_expected[k]));
    }
}

TEST_CASE("Cholesky solve symmetric positive definite matrix" * doctest::description("[fsb_linalg][fsb_linalg_sym_lt_chol]"))
{
    // Inputs
    constexpr size_t adim = 3U;
    constexpr size_t nrhs = 2U;
    const std::array<double_t, adim * adim> a_mat = {
        4.0, 12.0, -16.0,
        0.0, 37.0, -43.0,
        0.0, 0.0, 98.0
    };
    const std::array<double_t, nrhs * adim> b_vec = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    constexpr size_t work_len = adim * adim;
    std::array<double_t, work_len> work = {};
    // Expected
    const FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_ERROR_NONE;
    const std::array<double_t, nrhs * adim> x_vec_expected = {
        28.583333333333265, -7.666666666666648, 1.333333333333330, 142.333333333333030, -38.666666666666579, 6.333333333333320};
    // Process
    std::array<double_t, nrhs * adim> x_vec_actual = {};
    const FsbLinalgErrorType err_actual = fsb_linalg_cholesky_solve_array<adim, nrhs, work_len>(
        a_mat, b_vec, work, x_vec_actual);

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < (nrhs * adim); ++k)
    {
        REQUIRE(x_vec_actual[k] == FsbApprox(x_vec_expected[k]));
    }
}

TEST_CASE("Least squares solution underdetermined system" * doctest::description("[fsb_linalg][fsb_linalg_leastsquares_solve]"))
{
    // Inputs - Underdetermined system (fewer rows than columns)
    constexpr size_t rows = 2U;
    constexpr size_t columns = 3U;
    constexpr size_t nrhs = 1U;
    // A*x = b where A is 2x3, x is 3x1, b is 2x1
    const std::array<double_t, rows * columns> a_mat = {
        2.0, 1.0,
        1.0, 2.0,
        3.0, 1.0
    };
    const std::array<double_t, rows * nrhs> b_vec = {
        5.0,
        4.0
    };
    // Expected: Least-norm solution
    // For underdetermined systems, dgelsd returns the minimum norm solution
    const std::array<double_t, columns * nrhs> x_expected = {
        0.7142857142857137,
        1.257142857142857,
        0.7714285714285709
    };

    constexpr size_t work_len = 256U;
    std::array<double_t, work_len> work = {};
    FsbLinalgErrorType err_expected = EFSB_LAPACK_ERROR_NONE;

    // Process
    std::array<double_t, columns * nrhs> x_actual = {};
    const FsbLinalgErrorType err_actual = fsb_linalg_leastsquares_solve_array<rows, columns, nrhs, work_len>(
        a_mat, b_vec, work, x_actual);

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t idx = 0U; idx < columns; ++idx)
    {
        REQUIRE(x_actual[idx] == FsbApprox(x_expected[idx]));
    }
}

TEST_CASE("Least squares solution overdetermined system" * doctest::description("[fsb_linalg][fsb_linalg_leastsquares_solve]"))
{
    // Inputs - Overdetermined system (more rows than columns)
    constexpr size_t rows = 4U;
    constexpr size_t columns = 2U;
    constexpr size_t nrhs = 1U;
    // A*x = b where A is 4x2, x is 2x1, b is 4x1
    // This system will likely have no exact solution
    const std::array<double_t, rows * columns> a_mat = {
        1.0, 1.2, 2.0, 2.5,
        1.5, 2.0, 3.0, 3.5
    };
    const std::array<double_t, rows * nrhs> b_vec = {
        3.0,
        4.0,
        5.0,
        8.0
    };
    // Expected: Least squares solution (calculated independently)
    const std::array<double_t, columns * nrhs> x_expected = {
        3.8394793926247286,
        -0.5856832971800433
    };

    constexpr size_t work_len = 256U;
    std::array<double_t, work_len> work = {};
    FsbLinalgErrorType err_expected = EFSB_LAPACK_ERROR_NONE;

    // Process
    std::array<double_t, columns * nrhs> x_actual = {};
    const FsbLinalgErrorType err_actual = fsb_linalg_leastsquares_solve_array<rows, columns, nrhs, work_len>(
        a_mat, b_vec, work, x_actual);

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t idx = 0U; idx < columns; ++idx)
    {
        REQUIRE(x_actual[idx] == FsbApprox(x_expected[idx]));
    }
}

TEST_SUITE_END();
