#include <doctest/doctest.h>
#include <array>
#include "fsb_test_macros.h"
#include "fsb_linalg.h"

TEST_SUITE_BEGIN("linalg");

TEST_CASE("Singular Value Decomposition" * doctest::description("[fsb_linalg][fsb_linalg_svd]"))
{
    // Inputs
    const size_t rows = 3U;
    const size_t cols = 4U;
    const double_t a_mat[rows * cols] = {
        0.957166948242946, 0.485375648722841, 0.800280468888800,
        0.141886338627215, 0.421761282626275, 0.915735525189067,
        0.792207329559554, 0.959492426392903, 0.655740699156587,
        0.0357116785741896, 0.849129305868777, 0.933993247757551
    };
    const bool u_full = false;
    const bool v_full = false;
    constexpr size_t work_len = 256U;
    double_t work[work_len] = {};

    // Expected
    const double_t u_expected[rows * rows] = {
        -0.436686793704996, -0.585976373201268, -0.682595293166851,
        0.891072308003706, -0.177432382809533, -0.417741416954892,
        0.123672091082020, -0.790663923282465, 0.599629697652627
    };
    const double_t s_expected[rows] = {
        2.36519644590068, 0.790034423136396, 0.428179268650398
    };
    const double_t vt_expected[rows * cols] = {
        -0.527934375763356, 0.547410712881673, 0.500906453486051, -0.394968797595884,
        -0.418897921755130, 0.544580700083686, -0.573225910244715, 0.331300674476610,
        -0.624646104104312, -0.486515158692034, -0.644287129828739, -0.249683460765213
    };
    FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_ERROR_NONE;
    // Actual
    double_t u_actual[rows * rows] = {};
    double_t s_actual[rows] = {};
    double_t vt_actual[rows * cols] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    // Process
    FsbLinalgErrorType err_actual = fsb_linalg_svd(
        a_mat, rows, cols, u_full, v_full, work_len, work,
        u_actual, s_actual, vt_actual);

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
    for (size_t k = 0U; k < (rows * cols); ++k)
    {
        REQUIRE(vt_actual[k] == FsbApprox(vt_expected[k]));
    }
}

TEST_CASE("Singular Value Decomposition CPP Array" * doctest::description("[fsb_linalg][fsb_linalg_svd]"))
{
    // Inputs
    constexpr size_t rows = 3U;
    constexpr size_t cols = 4U;
    const std::array<double, rows * cols> a_mat = {
        0.957166948242946, 0.485375648722841, 0.800280468888800,
        0.141886338627215, 0.421761282626275, 0.915735525189067,
        0.792207329559554, 0.959492426392903, 0.655740699156587,
        0.0357116785741896, 0.849129305868777, 0.933993247757551
    };
    constexpr size_t work_len = 256U;
    std::array<double, work_len> work = {};
    const bool u_full = false;
    const bool v_full = false;

    // Expected
    const std::array<double, rows * rows> u_expected = {
        -0.436686793704996, -0.585976373201268, -0.682595293166851,
        0.891072308003706, -0.177432382809533, -0.417741416954892,
        0.123672091082020, -0.790663923282465, 0.599629697652627
    };
    const std::array<double, rows> s_expected = {
        2.36519644590068, 0.790034423136396, 0.428179268650398
    };
    const std::array<double, rows * cols> vt_expected = {
        -0.527934375763356, 0.547410712881673, 0.500906453486051, -0.394968797595884,
        -0.418897921755130, 0.544580700083686, -0.573225910244715, 0.331300674476610,
        -0.624646104104312, -0.486515158692034, -0.644287129828739, -0.249683460765213
    };
    FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_ERROR_NONE;

    // Actual
    std::array<double, rows * rows> u_actual = {};
    std::array<double, cols> s_actual = {};
    std::array<double, cols * cols> vt_actual = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    // Process
    const FsbLinalgErrorType err_actual = fsb_linalg_svd(
        a_mat.data(), rows, cols, u_full, v_full, work_len, work.data(),
        u_actual.data(), s_actual.data(), vt_actual.data());

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
    for (size_t k = 0U; k < (rows * cols); ++k)
    {
        REQUIRE(vt_actual[k] == FsbApprox(vt_expected[k]));
    }
}

TEST_CASE("Eigenvalue decomposition" * doctest::description("[fsb_linalg][fsb_linalg_matrix_eig]"))
{
    // Inputs
    const size_t adim = 3U;
    const double_t a_mat[adim * adim] = {
        0.792207329559554, 0.959492426392903, 0.655740699156587,
        0.0357116785741896, 0.849129305868777, 0.933993247757551,
        0.678735154857774, 0.757740130578333, 0.743132468124916
    };
    constexpr size_t work_len = 400U;
    double_t work[work_len] = {};

    // Expected
    const double_t eig_val_real_expected[adim] = {
        2.08316648761762, 0.150651307967817, 0.150651307967817
    };
    const double_t eig_val_imag_expected[adim] = {
       0.0, 0.234508337575288, -0.234508337575288
    };
    const double_t eig_vec_real_expected[adim * adim] = {
        0.358017325865748, 0.674696102475708, 0.645452371352083,
        0.616826825411530, -0.00999200711587476, -0.666934264156766,
        0.616826825411530, -0.00999200711587476, -0.666934264156766
    };
    const double_t eig_vec_imag_expected[adim * adim] = {
        0.0, 0.0, 0.0,
        0.244339279699821, -0.339001225567072, 0.0,
        -0.244339279699821, 0.339001225567072, 0.0
    };
    FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_ERROR_NONE;
    // Actual
    double_t eig_val_real_actual[adim] = {};
    double_t eig_val_imag_actual[adim] = {};
    double_t eig_vec_real_actual[adim * adim] = {};
    double_t eig_vec_imag_actual[adim * adim] = {};
    // Process
    FsbLinalgErrorType err_actual = fsb_linalg_matrix_eig(
        a_mat, adim, work_len, work,
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
    const size_t adim = 3U;
    const double_t a_mat[adim * adim] = {
        5745.0, -55.722221, 136.4,
        0.0, 7073.0, 49.635477,
        0.0, 0.0, 2614.0
    };
    constexpr size_t work_len = 128U;
    double_t work[work_len] = {};

    // Expected
    const double_t eig_val_expected[adim] = {
        2607.46319128460, 5748.76982107806, 7075.76698763734
    };
    const double_t eig_vec_expected[adim * adim] = {
        -0.0436361061779135, -0.0116483932490119, 0.998979581959685,
        0.998213049251589, 0.0403517891385511, 0.0440731371377369,
        0.0408239946780535, -0.999117634739791, -0.00986678318865923
    };
    FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_ERROR_NONE;
    // Actual
    double_t eig_val_actual[adim] = {};
    double_t eig_vec_actual[adim * adim] = {};
    // Process
    FsbLinalgErrorType err_actual = fsb_linalg_sym_lt_eig(
        a_mat, adim, work_len, work,
        eig_val_actual, eig_vec_actual);

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < adim; ++k)
    {
        REQUIRE(eig_val_actual[k] == FsbApprox(eig_val_expected[k]));
    }
    for (size_t k = 0U; k < (adim * adim); ++k)
    {
        REQUIRE(eig_vec_actual[k] == FsbApprox(eig_vec_expected[k]));
    }
}

TEST_CASE("Cholesky factorization of not positive definite matrix" * doctest::description("[fsb_linalg][fsb_linalg_sym_lt_chol]"))
{
    // Inputs
    const size_t adim = 3U;
    const double_t a_mat[adim * adim] = {
        0.814723686393179, 0.905791937075619, 0.126986816293506,
        0.0, 0.632359246225410, 0.0975404049994095,
        0.0, 0.0, 0.957506835434298
    };
    // Expected
    FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_NOT_POSITIVE_DEFINITE;
    // Actual
    double_t chol_actual[adim * adim] = {};
    // Process
    FsbLinalgErrorType err_actual = fsb_linalg_cholesky_decomposition(
        a_mat, adim, chol_actual);

    // Check result
    REQUIRE(err_actual == err_expected);
}

TEST_CASE("Cholesky factorization of symmetric positive definite matrix" * doctest::description("[fsb_linalg][fsb_linalg_sym_lt_chol]"))
{
    // Inputs
    const size_t adim = 3U;
    const double_t a_mat[adim * adim] = {
        1.814723686393179, 0.905791937075619, 0.126986816293506,
        0.0, 1.632359246225410, 0.0975404049994095,
        0.0, 0.0, 1.957506835434298
    };
    // Expected
    const double_t chol_expected[adim * adim] = {
        1.34711680502961, 0.672393020184847, 0.0942656314726287,
        0.0, 1.08639167551676, 0.0314406425635117,
        0.0, 0.0, 1.39557597863841
    };
    FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_ERROR_NONE;
    // Actual
    double_t chol_actual[adim * adim] = {};
    // Process
    FsbLinalgErrorType err_actual = fsb_linalg_cholesky_decomposition(
        a_mat, adim, chol_actual);

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
    const size_t adim = 3U;
    const size_t nrhs = 2U;
    const double_t a_mat[adim * adim] = {
        -4.0, 12.0, -16.0,
        0.0, 37.0, -43.0,
        0.0, 0.0, 98.0
    };
    const double_t b_vec[nrhs * adim] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    const size_t work_len = adim * adim;
    double_t work[work_len] = {};
    // Expected
    const FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_NOT_POSITIVE_DEFINITE;
    const double_t x_vec_expected[nrhs * adim] = {};
    // Process
    double_t x_vec_actual[nrhs * adim] = {};
    FsbLinalgErrorType err_actual = fsb_linalg_cholesky_solve(
        a_mat, b_vec, nrhs, adim, work_len, work, x_vec_actual);

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
    const size_t adim = 3U;
    const size_t nrhs = 2U;
    const double_t a_mat[adim * adim] = {
        4.0, 12.0, -16.0,
        0.0, 37.0, -43.0,
        0.0, 0.0, 98.0
    };
    const double_t b_vec[nrhs * adim] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    const size_t work_len = adim * adim;
    double_t work[work_len] = {};
    // Expected
    const FsbLinalgErrorType err_expected = FsbLinalgErrorType::EFSB_LAPACK_ERROR_NONE;
    const double_t x_vec_expected[nrhs * adim] = {
        28.583333333333265, -7.666666666666648, 1.333333333333330, 142.333333333333030, -38.666666666666579, 6.333333333333320};
    // Process
    double_t x_vec_actual[nrhs * adim] = {};
    FsbLinalgErrorType err_actual = fsb_linalg_cholesky_solve(
        a_mat, b_vec, nrhs, adim, work_len, work, x_vec_actual);

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
    const size_t rows = 2U;
    const size_t columns = 3U;
    const size_t nrhs = 1U;
    // A*x = b where A is 2x3, x is 3x1, b is 2x1
    const double_t a_mat[rows * columns] = {
        2.0, 1.0,
        1.0, 2.0,
        3.0, 1.0
    };
    const double_t b_vec[rows * nrhs] = {
        5.0,
        4.0
    };
    // Expected: Least-norm solution
    // For underdetermined systems, dgelsd returns the minimum norm solution
    const double_t x_expected[columns * nrhs] = {
        0.7142857142857137,
        1.257142857142857,
        0.7714285714285709
    };

    constexpr size_t work_len = 256U;
    double_t work[work_len] = {};
    FsbLinalgErrorType err_expected = EFSB_LAPACK_ERROR_NONE;

    // Process
    double_t x_actual[columns * nrhs] = {};
    FsbLinalgErrorType err_actual = fsb_linalg_leastsquares_solve(
        a_mat, rows, columns, b_vec, nrhs,
        work_len, work, x_actual);

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
    const size_t rows = 4U;
    const size_t columns = 2U;
    const size_t nrhs = 1U;
    // A*x = b where A is 4x2, x is 2x1, b is 4x1
    // This system will likely have no exact solution
    const double_t a_mat[rows * columns] = {
        1.0, 1.2, 2.0, 2.5,
        1.5, 2.0, 3.0, 3.5
    };
    const double_t b_vec[rows * nrhs] = {
        3.0,
        4.0,
        5.0,
        8.0
    };
    // Expected: Least squares solution (calculated independently)
    const double_t x_expected[columns * nrhs] = {
        3.8394793926247286,
        -0.5856832971800433
    };

    constexpr size_t work_len = 256U;
    double_t work[work_len] = {};
    FsbLinalgErrorType err_expected = EFSB_LAPACK_ERROR_NONE;

    // Process
    double_t x_actual[columns * nrhs] = {};
    FsbLinalgErrorType err_actual = fsb_linalg_leastsquares_solve(
        a_mat, rows, columns, b_vec, nrhs,
        work_len, work, x_actual);

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t idx = 0U; idx < columns; ++idx)
    {
        REQUIRE(x_actual[idx] == FsbApprox(x_expected[idx]));
    }
}

TEST_SUITE_END();
