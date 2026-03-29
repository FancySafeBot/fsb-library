#include <doctest/doctest.h>

#include "fsb_test_macros.h"
#include "fsb_linalg.h"

TEST_SUITE_BEGIN("linalg");

TEST_CASE("Singular Value Decomposition" * doctest::description("[fsb_linalg][fsb::linalg_svd]"))
{
    // Inputs
    constexpr size_t rows = 3U;
    constexpr size_t cols = 4U;
    const fsb::Array<rows * cols> a_mat = {{
        0.957166948242946, 0.485375648722841, 0.800280468888800,
        0.141886338627215, 0.421761282626275, 0.915735525189067,
        0.792207329559554, 0.959492426392903, 0.655740699156587,
        0.0357116785741896, 0.849129305868777, 0.933993247757551
    }};
    constexpr size_t work_len = 256U;
    fsb::Array<work_len> work = {};
    const bool u_full = false;
    const bool v_full = false;

    // Expected
    const fsb::Array<rows * rows> u_expected = {{
        -0.436686793704996, -0.585976373201268, -0.682595293166851,
        0.891072308003706, -0.177432382809533, -0.417741416954892,
        0.123672091082020, -0.790663923282465, 0.599629697652627
    }};
    const fsb::Array<FSB_MIN(rows, cols)> s_expected = {{
        2.36519644590068, 0.790034423136396, 0.428179268650398
    }};
    const fsb::Array<FSB_MIN(rows, cols) * cols> vt_expected = {{
        -0.527934375763356, 0.547410712881673, 0.500906453486051, -0.394968797595884,
        -0.418897921755130, 0.544580700083686, -0.573225910244715, 0.331300674476610,
        -0.624646104104312, -0.486515158692034, -0.644287129828739, -0.249683460765213
    }};
    fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;

    // Actual
    fsb::Array<rows * rows> u_actual = {};
    fsb::Array<FSB_MIN(rows, cols)> s_actual = {};
    fsb::Array<FSB_MIN(rows, cols) * cols> vt_actual = {};
    // Process
    const fsb::LinalgErrorType err_actual = fsb::linalg_svd(
        a_mat.active(rows * cols), rows, cols,
        u_full, v_full,
        work.active(work_len),
        u_actual.active(rows * rows),
        s_actual.active(FSB_MIN(rows, cols)),
        vt_actual.active(FSB_MIN(rows, cols) * cols));

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

TEST_CASE("Eigenvalue decomposition" * doctest::description("[fsb_linalg][fsb::linalg_matrix_eig]"))
{
    // Inputs
    constexpr size_t adim = 3U;
    const fsb::Array<adim * adim> a_mat = {{
        0.792207329559554, 0.959492426392903, 0.655740699156587,
        0.0357116785741896, 0.849129305868777, 0.933993247757551,
        0.678735154857774, 0.757740130578333, 0.743132468124916
    }};
    constexpr size_t work_len = 400U;
    fsb::Array<work_len> work = {};

    // Expected
    const fsb::Array<adim> eig_val_real_expected = {{
        2.08316648761762, 0.150651307967817, 0.150651307967817
    }};
    const fsb::Array<adim> eig_val_imag_expected = {{
       0.0, 0.234508337575288, -0.234508337575288
    }};
    const fsb::Array<adim * adim> eig_vec_real_expected = {{
        0.358017325865748, 0.674696102475708, 0.645452371352083,
        0.616826825411530, -0.00999200711587476, -0.666934264156766,
        0.616826825411530, -0.00999200711587476, -0.666934264156766
    }};
    const fsb::Array<adim * adim> eig_vec_imag_expected = {{
        0.0, 0.0, 0.0,
        0.244339279699821, -0.339001225567072, 0.0,
        -0.244339279699821, 0.339001225567072, 0.0
    }};
    fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;
    // Actual
    fsb::Array<adim> eig_val_real_actual = {};
    fsb::Array<adim> eig_val_imag_actual = {};
    fsb::Array<adim * adim> eig_vec_real_actual = {};
    fsb::Array<adim * adim> eig_vec_imag_actual = {};
    // Process
    const fsb::LinalgErrorType err_actual = fsb::linalg_matrix_eig(
        a_mat.active(adim * adim), adim,
        work.active(work_len),
        eig_val_real_actual.active(adim),
        eig_val_imag_actual.active(adim),
        eig_vec_real_actual.active(adim * adim),
        eig_vec_imag_actual.active(adim * adim));

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

TEST_CASE("Eigenvalue decomposition of symmetric positive definite" * doctest::description("[fsb_linalg][fsb::linalg_sym_lt_eig]"))
{
    // Inputs
    constexpr size_t adim = 3U;
    const fsb::Array<adim * adim> a_mat = {{
        5745.0, -55.722221, 136.4,
        0.0, 7073.0, 49.635477,
        0.0, 0.0, 2614.0
    }};
    constexpr size_t work_len = 128U;
    fsb::Array<work_len> work = {};

    // Expected
    const fsb::Array<adim> eig_val_expected = {{
        2607.46319128460, 5748.76982107806, 7075.76698763734
    }};
    const fsb::Array<adim * adim> eig_vec_expected = {{
        -0.0436361061779135, -0.0116483932490119, 0.998979581959685,
        0.998213049251589, 0.0403517891385511, 0.0440731371377369,
        0.0408239946780535, -0.999117634739791, -0.00986678318865923
    }};
    fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;
    // Actual
    fsb::Array<adim> eig_val_actual = {};
    fsb::Array<adim * adim> eig_vec_actual = {};
    // Process
    const fsb::LinalgErrorType err_actual = fsb::linalg_sym_lt_eig(
        a_mat.active(adim * adim), adim,
        work.active(work_len),
        eig_val_actual.active(adim),
        eig_vec_actual.active(adim * adim));

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

TEST_CASE("Cholesky factorization of not positive definite matrix" * doctest::description("[fsb_linalg][fsb::linalg_sym_lt_chol]"))
{
    // Inputs
    constexpr size_t adim = 3U;
    const fsb::Array<adim * adim> a_mat = {{
        0.814723686393179, 0.905791937075619, 0.126986816293506,
        0.0, 0.632359246225410, 0.0975404049994095,
        0.0, 0.0, 0.957506835434298
    }};
    // Expected
    fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::NOT_POSITIVE_DEFINITE;
    // Actual
    fsb::Array<adim * adim> chol_actual = {};
    // Process
    const fsb::LinalgErrorType err_actual = fsb::linalg_cholesky_decomposition(
        a_mat.active(adim * adim), adim,
        chol_actual.active(adim * adim));

    // Check result
    REQUIRE(err_actual == err_expected);
}

TEST_CASE("Cholesky factorization of symmetric positive definite matrix" * doctest::description("[fsb_linalg][fsb::linalg_sym_lt_chol]"))
{
    // Inputs
    constexpr size_t adim = 3U;
    const fsb::Array<adim * adim> a_mat = {{
        1.814723686393179, 0.905791937075619, 0.126986816293506,
        0.0, 1.632359246225410, 0.0975404049994095,
        0.0, 0.0, 1.957506835434298
    }};
    // Expected
    const fsb::Array<adim * adim> chol_expected = {{
        1.34711680502961, 0.672393020184847, 0.0942656314726287,
        0.0, 1.08639167551676, 0.0314406425635117,
        0.0, 0.0, 1.39557597863841
    }};
    fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;
    // Actual
    fsb::Array<adim * adim> chol_actual = {};
    // Process
    const fsb::LinalgErrorType err_actual = fsb::linalg_cholesky_decomposition(
        a_mat.active(adim * adim), adim,
        chol_actual.active(adim * adim));

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < (adim * adim); ++k)
    {
        REQUIRE(chol_actual[k] == FsbApprox(chol_expected[k]));
    }
}


TEST_CASE("Cholesky solve not positive definite matrix" * doctest::description("[fsb_linalg][fsb::linalg_sym_lt_chol]"))
{
    // Inputs
    constexpr size_t adim = 3U;
    constexpr size_t nrhs = 2U;
    const fsb::Array<adim * adim> a_mat = {{
        -4.0, 12.0, -16.0,
        0.0, 37.0, -43.0,
        0.0, 0.0, 98.0
    }};
    const fsb::Array<adim * nrhs> b_vec = {{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}};
    constexpr size_t work_len = adim * adim;
    fsb::Array<work_len> work = {};
    // Expected
    const fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::NOT_POSITIVE_DEFINITE;
    const fsb::Array<adim * nrhs> x_vec_expected = {};
    // Process
    fsb::Array<adim * nrhs> x_vec_actual = {};
    const fsb::LinalgErrorType err_actual = fsb::linalg_cholesky_solve(
        a_mat.active(adim * adim), adim,
        b_vec.active(adim * nrhs), nrhs,
        work.active(work_len),
        x_vec_actual.active(adim * nrhs));

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < (nrhs * adim); ++k)
    {
        REQUIRE(x_vec_actual[k] == FsbApprox(x_vec_expected[k]));
    }
}

TEST_CASE("Cholesky solve symmetric positive definite matrix" * doctest::description("[fsb_linalg][fsb::linalg_sym_lt_chol]"))
{
    // Inputs
    constexpr size_t adim = 3U;
    constexpr size_t nrhs = 2U;
    const fsb::Array<adim * adim> a_mat = {{
        4.0, 12.0, -16.0,
        0.0, 37.0, -43.0,
        0.0, 0.0, 98.0
    }};
    const fsb::Array<adim * nrhs> b_vec = {{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}};
    constexpr size_t work_len = adim * adim;
    fsb::Array<work_len> work = {};
    // Expected
    const fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;
    const fsb::Array<adim * nrhs> x_vec_expected = {{
        28.583333333333265, -7.666666666666648, 1.333333333333330, 142.333333333333030, -38.666666666666579, 6.333333333333320}};
    // Process
    fsb::Array<adim * nrhs> x_vec_actual = {};
    const fsb::LinalgErrorType err_actual = fsb::linalg_cholesky_solve(
        a_mat.active(adim * adim), adim,
        b_vec.active(adim * nrhs), nrhs,
        work.active(work_len),
        x_vec_actual.active(adim * nrhs));

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < (nrhs * adim); ++k)
    {
        REQUIRE(x_vec_actual[k] == FsbApprox(x_vec_expected[k]));
    }
}

TEST_CASE("Least squares solution underdetermined system" * doctest::description("[fsb_linalg][fsb::linalg_leastsquares_solve]"))
{
    // Inputs - Underdetermined system (fewer rows than columns)
    constexpr size_t rows = 2U;
    constexpr size_t columns = 3U;
    constexpr size_t nrhs = 1U;
    // A*x = b where A is 2x3, x is 3x1, b is 2x1
    const fsb::Array<rows * columns> a_mat = {{
        2.0, 1.0,
        1.0, 2.0,
        3.0, 1.0
    }};
    const fsb::Array<rows * nrhs> b_vec = {{
        5.0,
        4.0
    }};
    // Expected: Least-norm solution
    // For underdetermined systems, dgelsd returns the minimum norm solution
    const fsb::Array<columns * nrhs> x_expected = {{
        0.7142857142857137,
        1.257142857142857,
        0.7714285714285709
    }};

    constexpr size_t work_len = 256U;
    fsb::Array<work_len> work = {};
    fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;

    // Process
    fsb::Array<columns * nrhs> x_actual = {};
    const fsb::LinalgErrorType err_actual = fsb::linalg_leastsquares_solve(
        a_mat.active(rows * columns), rows, columns,
        b_vec.active(rows * nrhs), nrhs,
        work.active(work_len),
        x_actual.active(columns * nrhs));

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t idx = 0U; idx < columns; ++idx)
    {
        REQUIRE(x_actual[idx] == FsbApprox(x_expected[idx]));
    }
}

TEST_CASE("Least squares solution overdetermined system" * doctest::description("[fsb_linalg][fsb::linalg_leastsquares_solve]"))
{
    // Inputs - Overdetermined system (more rows than columns)
    constexpr size_t rows = 4U;
    constexpr size_t columns = 2U;
    constexpr size_t nrhs = 1U;
    // A*x = b where A is 4x2, x is 2x1, b is 4x1
    // This system will likely have no exact solution
    const fsb::Array<rows * columns> a_mat = {{
        1.0, 1.2, 2.0, 2.5,
        1.5, 2.0, 3.0, 3.5
    }};
    const fsb::Array<rows * nrhs> b_vec = {{
        3.0,
        4.0,
        5.0,
        8.0
    }};
    // Expected: Least squares solution (calculated independently)
    const fsb::Array<columns * nrhs> x_expected = {{
        3.8394793926247286,
        -0.5856832971800433
    }};

    constexpr size_t work_len = 256U;
    fsb::Array<work_len> work = {};
    fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;

    // Process
    fsb::Array<columns * nrhs> x_actual = {};
    const fsb::LinalgErrorType err_actual = fsb::linalg_leastsquares_solve(
        a_mat.active(rows * columns), rows, columns,
        b_vec.active(rows * nrhs), nrhs,
        work.active(work_len),
        x_actual.active(columns * nrhs));

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t idx = 0U; idx < columns; ++idx)
    {
        REQUIRE(x_actual[idx] == FsbApprox(x_expected[idx]));
    }
}

TEST_CASE("SVD typed container overload" * doctest::description("[fsb_linalg][fsb::linalg_svd_array][typed]"))
{
    // Oversized containers — only rows x cols are active
    constexpr size_t rows = 3U;
    constexpr size_t cols = 4U;
    constexpr size_t MaxR = 6U;
    constexpr size_t MaxC = 8U;
    constexpr size_t MaxWork = 512U;
    fsb::Array<MaxR * MaxC> a_mat = {};
    constexpr fsb::Real data[rows * cols] = {
        0.957166948242946, 0.485375648722841, 0.800280468888800,
        0.141886338627215, 0.421761282626275, 0.915735525189067,
        0.792207329559554, 0.959492426392903, 0.655740699156587,
        0.0357116785741896, 0.849129305868777, 0.933993247757551
    };
    for (size_t k = 0U; k < rows * cols; ++k)
    {
        a_mat[k] = data[k];
    }
    constexpr size_t work_len = 256U;
    fsb::Array<MaxWork> work = {};
    const bool u_full = false;
    const bool v_full = false;
    constexpr size_t s = FSB_MIN(rows, cols);

    // Expected
    const fsb::Array<s> s_expected = {{2.36519644590068, 0.790034423136396, 0.428179268650398}};
    const fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;

    // Actual — containers larger than the active block
    fsb::Array<MaxR * MaxR> u_actual = {};
    fsb::Array<MaxR> sing_val_actual = {};
    fsb::Array<MaxR * MaxC> vt_actual = {};
    const fsb::LinalgErrorType err_actual = fsb::linalg_svd_array(
        a_mat, rows, cols,
        u_full, v_full,
        work, work_len,
        u_actual,
        sing_val_actual, s,
        vt_actual);

    // Check result
    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < s; ++k)
    {
        REQUIRE(sing_val_actual[k] == FsbApprox(s_expected[k]));
    }
}

TEST_CASE("Eigenvalue decomposition typed container overload" * doctest::description("[fsb_linalg][fsb::linalg_matrix_eig_array][typed]"))
{
    constexpr size_t adim = 3U;
    constexpr size_t MaxDim = 6U;
    constexpr size_t MaxWork = 512U;
    fsb::Array<MaxDim * MaxDim> a_mat = {};
    constexpr fsb::Real data[adim * adim] = {
        0.792207329559554, 0.959492426392903, 0.655740699156587,
        0.0357116785741896, 0.849129305868777, 0.933993247757551,
        0.678735154857774, 0.757740130578333, 0.743132468124916
    };
    for (size_t k = 0U; k < adim * adim; ++k)
    {
        a_mat[k] = data[k];
    }
    constexpr size_t work_len = 400U;
    fsb::Array<MaxWork> work = {};

    // Expected
    const fsb::Array<adim> eig_val_real_expected = {{2.08316648761762, 0.150651307967817, 0.150651307967817}};
    const fsb::Array<adim> eig_val_imag_expected = {{0.0, 0.234508337575288, -0.234508337575288}};
    const fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;

    // Actual — oversized containers
    fsb::Array<MaxDim> val_real = {};
    fsb::Array<MaxDim> val_imag = {};
    fsb::Array<MaxDim * MaxDim> vec_real = {};
    fsb::Array<MaxDim * MaxDim> vec_imag = {};
    const fsb::LinalgErrorType err_actual = fsb::linalg_matrix_eig_array(
        a_mat, adim,
        work, work_len,
        val_real, val_imag,
        vec_real, vec_imag);

    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < adim; ++k)
    {
        REQUIRE(val_real[k] == FsbApprox(eig_val_real_expected[k]));
        REQUIRE(val_imag[k] == FsbApprox(eig_val_imag_expected[k]));
    }
}

TEST_CASE("Symmetric eigenvalue decomposition typed container overload" * doctest::description("[fsb_linalg][fsb::linalg_sym_lt_eig_array][typed]"))
{
    constexpr size_t adim = 3U;
    constexpr size_t MaxDim = 6U;
    constexpr size_t MaxWork = 256U;
    fsb::Array<MaxDim * MaxDim> a_mat = {};
    constexpr fsb::Real data[adim * adim] = {
        5745.0, -55.722221, 136.4,
        0.0, 7073.0, 49.635477,
        0.0, 0.0, 2614.0
    };
    for (size_t k = 0U; k < adim * adim; ++k)
    {
        a_mat[k] = data[k];
    }
    constexpr size_t work_len = 128U;
    fsb::Array<MaxWork> work = {};

    const fsb::Array<adim> eig_val_expected = {{2607.46319128460, 5748.76982107806, 7075.76698763734}};
    const fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;

    fsb::Array<MaxDim> val_actual = {};
    fsb::Array<MaxDim * MaxDim> vec_actual = {};
    const fsb::LinalgErrorType err_actual = fsb::linalg_sym_lt_eig_array(
        a_mat, adim,
        work, work_len,
        val_actual, vec_actual);

    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < adim; ++k)
    {
        REQUIRE(val_actual[k] == FsbApprox(eig_val_expected[k]));
    }
}

TEST_CASE("Cholesky factorization not positive definite typed container overload" * doctest::description("[fsb_linalg][fsb::linalg_cholesky_decomposition_array][typed]"))
{
    constexpr size_t adim = 3U;
    constexpr size_t MaxDim = 6U;
    fsb::Array<MaxDim * MaxDim> a_mat = {};
    constexpr fsb::Real data[adim * adim] = {
        0.814723686393179, 0.905791937075619, 0.126986816293506,
        0.0, 0.632359246225410, 0.0975404049994095,
        0.0, 0.0, 0.957506835434298
    };
    for (size_t k = 0U; k < adim * adim; ++k)
    {
        a_mat[k] = data[k];
    }
    const fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::NOT_POSITIVE_DEFINITE;

    fsb::Array<MaxDim * MaxDim> chol_actual = {};
    const fsb::LinalgErrorType err_actual = fsb::linalg_cholesky_decomposition_array(
        a_mat, adim, chol_actual);

    REQUIRE(err_actual == err_expected);
}

TEST_CASE("Cholesky factorization positive definite typed container overload" * doctest::description("[fsb_linalg][fsb::linalg_cholesky_decomposition_array][typed]"))
{
    constexpr size_t adim = 3U;
    constexpr size_t MaxDim = 6U;
    fsb::Array<MaxDim * MaxDim> a_mat = {};
    constexpr fsb::Real data[adim * adim] = {
        1.814723686393179, 0.905791937075619, 0.126986816293506,
        0.0, 1.632359246225410, 0.0975404049994095,
        0.0, 0.0, 1.957506835434298
    };
    for (size_t k = 0U; k < adim * adim; ++k)
    {
        a_mat[k] = data[k];
    }
    const fsb::Array<adim * adim> chol_expected = {{
        1.34711680502961, 0.672393020184847, 0.0942656314726287,
        0.0, 1.08639167551676, 0.0314406425635117,
        0.0, 0.0, 1.39557597863841
    }};
    const fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;

    fsb::Array<MaxDim * MaxDim> chol_actual = {};
    const fsb::LinalgErrorType err_actual = fsb::linalg_cholesky_decomposition_array(
        a_mat, adim, chol_actual);

    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < adim * adim; ++k)
    {
        REQUIRE(chol_actual[k] == FsbApprox(chol_expected[k]));
    }
}

TEST_CASE("Cholesky solve positive definite typed container overload" * doctest::description("[fsb_linalg][fsb::linalg_cholesky_solve_array][typed]"))
{
    constexpr size_t adim = 3U;
    constexpr size_t nrhs = 2U;
    constexpr size_t MaxDim = 6U;
    constexpr size_t MaxNrhs = 4U;
    constexpr size_t MaxWork = 64U;
    fsb::Array<MaxDim * MaxDim> a_mat = {};
    constexpr fsb::Real a_data[adim * adim] = {
        4.0, 12.0, -16.0,
        0.0, 37.0, -43.0,
        0.0, 0.0, 98.0
    };
    for (size_t k = 0U; k < adim * adim; ++k)
    {
        a_mat[k] = a_data[k];
    }
    fsb::Array<MaxDim * MaxNrhs> b_vec = {};
    constexpr fsb::Real b_data[adim * nrhs] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    for (size_t k = 0U; k < adim * nrhs; ++k)
    {
        b_vec[k] = b_data[k];
    }
    constexpr size_t work_len = adim * adim;
    fsb::Array<MaxWork> work = {};

    const fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;
    const fsb::Array<adim * nrhs> x_vec_expected = {{
        28.583333333333265, -7.666666666666648, 1.333333333333330,
        142.333333333333030, -38.666666666666579, 6.333333333333320
    }};

    fsb::Array<MaxDim * MaxNrhs> x_vec_actual = {};
    const fsb::LinalgErrorType err_actual = fsb::linalg_cholesky_solve_array(
        a_mat, adim,
        b_vec, nrhs,
        work, work_len,
        x_vec_actual);

    REQUIRE(err_actual == err_expected);
    for (size_t k = 0U; k < adim * nrhs; ++k)
    {
        REQUIRE(x_vec_actual[k] == FsbApprox(x_vec_expected[k]));
    }
}

TEST_CASE("Least squares underdetermined typed container overload" * doctest::description("[fsb_linalg][fsb::linalg_leastsquares_solve_array][typed]"))
{
    constexpr size_t rows = 2U;
    constexpr size_t columns = 3U;
    constexpr size_t nrhs = 1U;
    constexpr size_t MaxR = 6U;
    constexpr size_t MaxC = 8U;
    constexpr size_t MaxNrhs = 4U;
    constexpr size_t MaxWork = 512U;
    fsb::Array<MaxR * MaxC> a_mat = {};
    constexpr fsb::Real a_data[rows * columns] = {
        2.0, 1.0,
        1.0, 2.0,
        3.0, 1.0
    };
    for (size_t k = 0U; k < rows * columns; ++k)
    {
        a_mat[k] = a_data[k];
    }
    fsb::Array<MaxR * MaxNrhs> b_vec = {};
    b_vec[0] = 5.0;
    b_vec[1] = 4.0;
    constexpr size_t work_len = 256U;
    fsb::Array<MaxWork> work = {};
    const fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;
    const fsb::Array<columns * nrhs> x_expected = {{
        0.7142857142857137, 1.257142857142857, 0.7714285714285709
    }};

    fsb::Array<MaxC * MaxNrhs> x_actual = {};
    const fsb::LinalgErrorType err_actual = fsb::linalg_leastsquares_solve_array(
        a_mat, rows, columns,
        b_vec, nrhs,
        work, work_len,
        x_actual);

    REQUIRE(err_actual == err_expected);
    for (size_t idx = 0U; idx < columns; ++idx)
    {
        REQUIRE(x_actual[idx] == FsbApprox(x_expected[idx]));
    }
}

TEST_CASE("Least squares overdetermined typed container overload" * doctest::description("[fsb_linalg][fsb::linalg_leastsquares_solve_array][typed]"))
{
    constexpr size_t rows = 4U;
    constexpr size_t columns = 2U;
    constexpr size_t nrhs = 1U;
    constexpr size_t MaxR = 8U;
    constexpr size_t MaxC = 6U;
    constexpr size_t MaxNrhs = 4U;
    constexpr size_t MaxWork = 512U;
    fsb::Array<MaxR * MaxC> a_mat = {};
    constexpr fsb::Real a_data[rows * columns] = {
        1.0, 1.2, 2.0, 2.5,
        1.5, 2.0, 3.0, 3.5
    };
    for (size_t k = 0U; k < rows * columns; ++k)
    {
        a_mat[k] = a_data[k];
    }
    fsb::Array<MaxR * MaxNrhs> b_vec = {};
    b_vec[0] = 3.0;
    b_vec[1] = 4.0;
    b_vec[2] = 5.0;
    b_vec[3] = 8.0;
    constexpr size_t work_len = 256U;
    fsb::Array<MaxWork> work = {};
    const fsb::LinalgErrorType err_expected = fsb::LinalgErrorType::ERROR_NONE;
    const fsb::Array<columns * nrhs> x_expected = {{
        3.8394793926247286, -0.5856832971800433
    }};

    fsb::Array<MaxC * MaxNrhs> x_actual = {};
    const fsb::LinalgErrorType err_actual = fsb::linalg_leastsquares_solve_array(
        a_mat, rows, columns,
        b_vec, nrhs,
        work, work_len,
        x_actual);

    REQUIRE(err_actual == err_expected);
    for (size_t idx = 0U; idx < columns; ++idx)
    {
        REQUIRE(x_actual[idx] == FsbApprox(x_expected[idx]));
    }
}

TEST_SUITE_END();
