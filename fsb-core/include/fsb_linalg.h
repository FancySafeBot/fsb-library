#pragma once
#include <algorithm>
#include <cstdint>
#include <math.h>
#include <array>
#include <cstddef>

#include "openblas/lapack.h"
#include "fsb_types.h"

/**
 * @defgroup LinearAlgebra Linear Algebra
 * @brief Linear algebra utility for matrix and vector computations
 *
 * @{
 */

#define FSB_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define FSB_MIN(a, b) (((a) < (b)) ? (a) : (b))

namespace fsb {

/**
 * @brief Error codes for linear algebra functions
 */
enum class LinalgErrorType : int
{
    /**
     * @brief No error
     */
    ERROR_NONE = 0,
    /**
     * @brief Input value error
     */
    ERROR_INPUT = 1,
    /**
     * @brief Not enough memory
     */
    ERROR_MEMORY = 2,
    /**
     * @brief Solution did not converge
     */
    ERROR_CONVERGE = 3,
    /**
     * @brief Work query failed
     */
    ERROR_QUERY = 4,
    /**
     * @brief Matrix not positive definite
     */
    NOT_POSITIVE_DEFINITE = 5,
    /**
     * @brief Input matrix is singular
     */
    SINGULAR = 6,
    ERROR_NOT_FULL_RANK = 7, ///< Matrix is not full rank
};

/**
 * @brief Eigenvalue decomposition for symmetric lower triangular matrix
 *
 * @param[in]    mat   input matrix (sxs)
 * @param[in]    dim   matrix A dimension
 * @param[in]  work_len    buffer length
 * @param[in,out]  work    work buffer
 * @param[out]   val eigenvalues (sx1)
 * @param[out]   vec column eigenvectors (sxs)
 *
 * @return Error code
 */
LinalgErrorType linalg_sym_lt_eig(
    const Real mat[], size_t dim, size_t work_len, Real work[],
    Real val[], Real vec[]);

/**
 * @brief Cholesky factorization for symmetric positive definite matrix
 *
 * @param mat Input matrix (dim x dim)
 * @param dim Matrix A dimension s
 * @param mat_chol Cholesky factorization of input matrix A (dim x dim)
 * @return Error code
 */
LinalgErrorType linalg_cholesky_decomposition(
    const Real mat[], size_t dim, Real mat_chol[]);

/**
 * @brief Cholesky solve
 *
 * square symmetric positive-definite matrix cholesky solve.
 * solve for b from x = A b
 * Variable x is overwritten by b on exit
 *
 * @param[in] mat Input matrix (dim x dim)
 * @param[in] b_vec Right-hand side vector (dim x nrhs)
 * @param[in] nrhs Number of right-hand side vectors
 * @param[in] dim Matrix dimension
 * @param[in] work_len Work buffer length
 * @param[in,out] work Work buffer
 * @param[out] x_vec Solution vector (dim x nrhs)
 * @return Error code
 */
LinalgErrorType linalg_cholesky_solve(const Real mat[], const Real b_vec[],
                                             size_t nrhs, size_t dim,
                                             size_t work_len, Real work[],
                                             Real x_vec[]);

/**
 * @brief Check if lower triangular symmetric matrix is positive definite
 *
 * @param[in] mat Input lower triangular matrix (sxs)
 * @param[in] dim Matrix A dimension
 * @param[in] work_len Work vector length
 * @param[in,out] work Work vector
 * @return true if matrix is positive definite, false otherwise
 */
bool linalg_is_posdef(const Real mat[], size_t dim, size_t work_len, Real work[]);

/**
 * @biref Solve linear system of equations with a square matrix
 *
 * @param mat Matrix to solve (dim x dim)
 * @param y_vec Right-hand side vector (dim x nrhs)
 * @param nrhs Number of right-hand side vectors
 * @param dim Matrix A dimension (s)
 * @param work_len Work vector length
 * @param iwork_len Integer work vector length
 * @param work Work vector
 * @param iwork Integer work vector
 * @param x_vec Output Solution vector (dim x nrhs)
 * @return Error code
 */
LinalgErrorType linalg_matrix_sqr_solve(
    const Real mat[], const Real y_vec[], size_t nrhs, size_t dim, size_t work_len,
    size_t iwork_len, Real work[], lapack_int iwork[], Real x_vec[]);

/**
 * @brief Inverse of a matrix where number of columns are great er than or equal to number of rows
 *
 * Work length must be at least rows * MIN(rows, columns) + MIN(rows, columns) + MIN(rows, columns) * columns
 * and additional length required by the linalg_svd function.
 *
 *  @param mat Input matrix (rows x columns)
 *  @param rows Number of rows in matrix
 *  @param columns Number of columns in matrix
 *  @param work_len Length of work vector
 *  @param work Work vector
 *  @param inv_mat Output inverse matrix (columns x rows)
 *
 *  @return Error code
 */
LinalgErrorType linalg_pseudoinverse(const Real mat[], size_t rows, size_t columns, size_t work_len, Real work[], Real inv_mat[]);

/**
 * @brief Solve overdetermined or underdetermined system using least squares
 *
 * Solves min ||A*X - B|| for X where A is an M-by-N matrix using the
 * singular value decomposition (SVD) approach. This method handles rank-deficient
 * least squares problems more robustly than the QR approach.
 *
 * @param mat Input matrix A (rows x columns)
 * @param rows Number of rows in A
 * @param columns Number of columns in A
 * @param b_vec Right-hand side matrix B (rows x nrhs)
 * @param nrhs Number of right-hand side vectors
 * @param work_len Length of work vector
 * @param work Work vector
 * @param x_vec Output solution matrix X (columns x nrhs).
 * @return Error code
 */
LinalgErrorType linalg_leastsquares_solve(
    const Real mat[], size_t rows, size_t columns, const Real b_vec[], size_t nrhs,
    size_t work_len, Real work[], Real x_vec[]);

/**
 * @brief SVD decomposition
 *
 * u_full and v_full affect the output matrices as follows:
 * if (m < n) then s = m
 * else s = n
 * if (u_full, v_full) == (0, 0) then U(m,s), S(s,s), VT(s,n)
 * if (u_full, v_full) == (1, 0) then U(m,m), S(m,s), VT(s,n)
 * if (u_full, v_full) == (0, 1) then U(m,s), S(s,n), VT(n,n)
 * if (u_full, v_full) == (1, 1) then U(m,m), S(m,n), VT(n,n)
 *
 * @param[in]  mat      matrix to perform SVD
 * @param[in]  rows      rows of A
 * @param[in]  cols      columns of A
 * @param[in]  u_full  Boolean for full U matrix
 * @param[in]  v_full  Boolean for full V matrix
 * @param[in]  work_len    buffer
 * @param[in,out]  work    buffer
 * @param[out] unitary_u      Unitary matrix U (m x m)
 * @param[out] sing_val      Singular values array S (s x 1)
 * @param[out] unitary_vt     Transpose of unitary matrix V (n x n)
 */
inline LinalgErrorType linalg_svd(
    Span<const Real> mat, size_t rows, size_t cols,
    bool u_full, bool v_full,
    Span<Real> work,
    Span<Real> unitary_u,
    Span<Real> sing_val,
    Span<Real> unitary_vt)
{
    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;

    if ((rows == 0U) || (cols == 0U) || (cols >= (static_cast<size_t>(INT32_MAX) / rows)))
    {
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else if (work.size() < (rows * cols))
    {
        // not enough buffer for copy of A
        retval = LinalgErrorType::ERROR_MEMORY;
    }
    else
    {
        const size_t a_len = rows * cols;
        Span<Real> a_mat_tmp = work.active(a_len);
        const size_t work_partial_len = a_len;

        const char* u_opt = (u_full ? "A" : "S");
        const char* v_opt = (v_full ? "A" : "S");
        const size_t u_opt_size = 1U;
        const size_t v_opt_size = 1U;

        // copy A to buffer
        std::copy_n(mat.data(), a_len, a_mat_tmp.data());

        const lapack_int l_m = static_cast<lapack_int>(rows);
        const lapack_int l_n = static_cast<lapack_int>(cols);
        const lapack_int ldvt = static_cast<lapack_int>(((rows < cols) && !v_full) ? rows : cols);
        Real work_query = 0.0;
        lapack_int lwork = -1;
        lapack_int info = 0;
        dgesvd_(u_opt, v_opt, &l_m, &l_n, a_mat_tmp.data(), &l_m, sing_val.data(), unitary_u.data(), &l_m, unitary_vt.data(), &ldvt, &work_query, &lwork, &info, u_opt_size, v_opt_size);
        lwork = static_cast<lapack_int>(work_query);
        const auto s_lwork = static_cast<size_t>(lwork);

        if (info != 0)
        {
            /* dgesvd work query failed */
            retval = LinalgErrorType::ERROR_QUERY;
        }
        else if (work.size() < (work_partial_len + s_lwork))
        {
            // not enough buffer for svd operation
            retval = LinalgErrorType::ERROR_MEMORY;
        }
        else
        {
            // allocate work
            Span<Real> svd_work = work.active(s_lwork, work_partial_len);

            /* perform SVD */
            dgesvd_(u_opt, v_opt, &l_m, &l_n, a_mat_tmp.data(), &l_m, sing_val.data(), unitary_u.data(), &l_m, unitary_vt.data(), &ldvt, svd_work.data(), &lwork, &info, u_opt_size, v_opt_size);
            if (info < 0)
            {
                // Some input error
                retval = LinalgErrorType::ERROR_INPUT;
            }
            else if (info > 0)
            {
                // DBDSQR did not converge (see DGESVD)
                retval = LinalgErrorType::ERROR_CONVERGE;
            }
            else
            {
                // success
            }
        }
    }
    return retval;
}

inline LinalgErrorType linalg_matrix_eig(
    Span<const Real> mat, size_t dim,
    Span<Real> work,
    Span<Real> val_real,
    Span<Real> val_imag,
    Span<Real> vec_real,
    Span<Real> vec_imag)
{
    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;
    if ((dim == 0U) || (dim >= (static_cast<size_t>(INT32_MAX) / dim)))
    {
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else if (work.size() < (dim * dim))
    {
        // not enough buffer for copy of A
        retval = LinalgErrorType::ERROR_MEMORY;
    }
    else
    {
        const size_t a_len = dim * dim;
        Span<Real> a_mat_tmp = work.active(a_len);
        const size_t work_partial_len = a_len;

        // copy A to buffer
        std::copy_n(mat.data(), a_len, a_mat_tmp.data());

        // work query inputs
        const char* left_vec_opt = "N";
        const char* right_vec_opt = "V";
        const size_t left_opt_size = 1U;
        const size_t right_opt_size = 1U;
        const lapack_int l_s = static_cast<lapack_int>(dim);
        const lapack_int lvd = 1;
        Real work_query = 0.0;
        lapack_int lwork = -1;
        lapack_int info = 0;

        // query work size
        dgeev_(left_vec_opt, right_vec_opt,
               &l_s, a_mat_tmp.data(), &l_s,
               val_real.data(), val_imag.data(),
               nullptr, &lvd,
               vec_real.data(), &l_s,
               &work_query, &lwork, &info,
               left_opt_size, right_opt_size);
        lwork = static_cast<lapack_int>(work_query);
        const size_t s_lwork = static_cast<size_t>(lwork);

        if (info != 0)
        {
            /* dgeev_ work query failed */
            retval = LinalgErrorType::ERROR_QUERY;
        }
        else if (work.size() < (work_partial_len + s_lwork))
        {
            // not enough buffer for operation
            retval = LinalgErrorType::ERROR_MEMORY;
        }
        else
        {
            Span<Real> eig_work = work.active(s_lwork, work_partial_len);

            /* perform decomposition */
            dgeev_(left_vec_opt, right_vec_opt,
                &l_s, a_mat_tmp.data(), &l_s,
                val_real.data(), val_imag.data(),
                nullptr, &lvd,
                vec_real.data(), &l_s,
                eig_work.data(), &lwork, &info,
                left_opt_size, right_opt_size);

            if (info < 0)
            {
                // Some input error
                retval = LinalgErrorType::ERROR_INPUT;
            }
            else if (info > 0)
            {
                // did not converge (see dgeev)
                retval = LinalgErrorType::ERROR_CONVERGE;
            }
            else
            {
                /* populate vec_imag */
                size_t idx = 0U;
                while (idx < dim)
                {
                    if (val_imag[idx] > 0.0)
                    {
                        /* increment */
                        idx++;
                        if (idx < dim)
                        {
                            /* complex conjugate pairs */
                            for (size_t jdx = 0U; jdx < dim; ++jdx)
                            {
                                vec_imag[(idx - 1U) * dim + jdx] = vec_real[idx * dim + jdx];
                                vec_imag[idx * dim + jdx] = -vec_real[idx * dim + jdx];
                                vec_real[idx * dim + jdx] = vec_real[(idx - 1U) * dim + jdx];
                            }
                        }
                    }
                    else
                    {
                        /* no imaginary vector */
                        for (size_t jdx = 0U; jdx < dim; ++jdx)
                        {
                            vec_imag[idx * dim + jdx] = 0.0;
                        }
                    }
                    /* increment */
                    idx++;
                }
            }
        }
    }
    return retval;
}

inline LinalgErrorType linalg_sym_lt_eig_array(
    Span<const Real> mat, size_t dim,
    Span<Real> work,
    Span<Real> val,
    Span<Real> vec)
{
    return linalg_sym_lt_eig(
        mat.data(), dim, work.size(), work.data(),
        val.data(), vec.data());
}

inline LinalgErrorType linalg_cholesky_decomposition_array(
    Span<const Real> mat, size_t dim,
    Span<Real> mat_chol)
{
    return linalg_cholesky_decomposition(mat.data(), dim, mat_chol.data());
}

inline bool linalg_is_posdef_array(
    Span<const Real> mat, size_t dim,
    Span<Real> work)
{
    return linalg_is_posdef(mat.data(), dim, work.size(), work.data());
}

inline LinalgErrorType linalg_cholesky_solve_array(
    Span<const Real> mat, size_t dim,
    Span<const Real> b_vec, size_t nrhs,
    Span<Real> work,
    Span<Real> x_vec)
{
    return linalg_cholesky_solve(
        mat.data(), b_vec.data(),
        nrhs, dim,
        work.size(), work.data(),
        x_vec.data());
}

inline LinalgErrorType linalg_matrix_sqr_solve_array(
    Span<const Real> mat, size_t dim,
    Span<const Real> y_vec, size_t nrhs,
    Span<Real> work,
    Span<lapack_int> iwork,
    Span<Real> x_vec)
{
    return linalg_matrix_sqr_solve(
        mat.data(), y_vec.data(),
        nrhs, dim,
        work.size(), iwork.size(),
        work.data(), iwork.data(),
        x_vec.data());
}

inline LinalgErrorType linalg_pseudoinverse_array(
    Span<const Real> mat, size_t rows, size_t cols,
    Span<Real> work,
    Span<Real> inv_mat)
{
    return linalg_pseudoinverse(
        mat.data(), rows, cols,
        work.size(), work.data(),
        inv_mat.data());
}

inline LinalgErrorType linalg_leastsquares_solve_array(
    Span<const Real> mat, size_t rows, size_t cols,
    Span<const Real> b_vec, size_t nrhs,
    Span<Real> work,
    Span<Real> x_vec)
{
    return linalg_leastsquares_solve(
        mat.data(), rows, cols,
        b_vec.data(), nrhs,
        work.size(), work.data(),
        x_vec.data());
}

/**
 * @brief Typed container wrappers — accept Array with explicit runtime
 * sizes, call .active() to form Span views, then forward to the Span-based overloads above.
 */
template <size_t MaxMat, size_t MaxWorkLen, size_t MaxU, size_t MaxS, size_t MaxVT>
inline LinalgErrorType linalg_svd_array(
    const Array<MaxMat>& mat, size_t rows, size_t cols,
    bool u_full, bool v_full,
    Array<MaxWorkLen>& work, size_t work_len,
    Array<MaxU>& unitary_u,
    Array<MaxS>& sing_val, size_t s_dim,
    Array<MaxVT>& unitary_vt)
{
    return linalg_svd(
        mat.active(rows * cols), rows, cols,
        u_full, v_full,
        work.active(work_len),
        unitary_u.active(rows * rows),
        sing_val.active(s_dim),
        unitary_vt.active(s_dim * cols));
}

template <size_t MaxMat, size_t MaxWorkLen, size_t MaxEig, size_t MaxVec>
inline LinalgErrorType linalg_matrix_eig_array(
    const Array<MaxMat>& mat, size_t dim,
    Array<MaxWorkLen>& work, size_t work_len,
    Array<MaxEig>& val_real,
    Array<MaxEig>& val_imag,
    Array<MaxVec>& vec_real,
    Array<MaxVec>& vec_imag)
{
    return linalg_matrix_eig(
        mat.active(dim * dim), dim,
        work.active(work_len),
        val_real.active(dim),
        val_imag.active(dim),
        vec_real.active(dim * dim),
        vec_imag.active(dim * dim));
}

template <size_t MaxMat, size_t MaxWorkLen, size_t MaxEig, size_t MaxVec>
inline LinalgErrorType linalg_sym_lt_eig_array(
    const Array<MaxMat>& mat, size_t dim,
    Array<MaxWorkLen>& work, size_t work_len,
    Array<MaxEig>& val,
    Array<MaxVec>& vec)
{
    return linalg_sym_lt_eig_array(
        mat.active(dim * dim), dim,
        work.active(work_len),
        val.active(dim),
        vec.active(dim * dim));
}

template <size_t MaxMat>
inline LinalgErrorType linalg_cholesky_decomposition_array(
    const Array<MaxMat>& mat, size_t dim,
    Array<MaxMat>& mat_chol)
{
    return linalg_cholesky_decomposition_array(
        mat.active(dim * dim), dim,
        mat_chol.active(dim * dim));
}

template <size_t MaxMat, size_t MaxWorkLen>
inline bool linalg_is_posdef_array(
    const Array<MaxMat>& mat, size_t dim,
    Array<MaxWorkLen>& work, size_t work_len)
{
    return linalg_is_posdef_array(mat.active(dim * dim), dim, work.active(work_len));
}

template <size_t MaxMat, size_t MaxVec, size_t MaxWorkLen>
inline LinalgErrorType linalg_cholesky_solve_array(
    const Array<MaxMat>& mat, size_t dim,
    const Array<MaxVec>& b_vec, size_t nrhs,
    Array<MaxWorkLen>& work, size_t work_len,
    Array<MaxVec>& x_vec)
{
    return linalg_cholesky_solve_array(
        mat.active(dim * dim), dim,
        b_vec.active(dim * nrhs), nrhs,
        work.active(work_len),
        x_vec.active(dim * nrhs));
}

template <size_t MaxMat, size_t MaxVec, size_t MaxWorkLen, size_t MaxIWorkLen>
inline LinalgErrorType linalg_matrix_sqr_solve_array(
    const Array<MaxMat>& mat, size_t dim,
    const Array<MaxVec>& y_vec, size_t nrhs,
    Array<MaxWorkLen>& work, size_t work_len,
    Array<MaxIWorkLen, lapack_int>& iwork, size_t iwork_len,
    Array<MaxVec>& x_vec)
{
    return linalg_matrix_sqr_solve_array(
        mat.active(dim * dim), dim,
        y_vec.active(dim * nrhs), nrhs,
        work.active(work_len),
        iwork.active(iwork_len),
        x_vec.active(dim * nrhs));
}

template <size_t MaxMat, size_t MaxInv, size_t MaxWorkLen>
inline LinalgErrorType linalg_pseudoinverse_array(
    const Array<MaxMat>& mat, size_t rows, size_t cols,
    Array<MaxWorkLen>& work, size_t work_len,
    Array<MaxInv>& inv_mat)
{
    return linalg_pseudoinverse_array(
        mat.active(rows * cols), rows, cols,
        work.active(work_len),
        inv_mat.active(cols * rows));
}

template <size_t MaxMat, size_t MaxB, size_t MaxX, size_t MaxWorkLen>
inline LinalgErrorType linalg_leastsquares_solve_array(
    const Array<MaxMat>& mat, size_t rows, size_t cols,
    const Array<MaxB>& b_vec, size_t nrhs,
    Array<MaxWorkLen>& work, size_t work_len,
    Array<MaxX>& x_vec)
{
    return linalg_leastsquares_solve_array(
        mat.active(rows * cols), rows, cols,
        b_vec.active(rows * nrhs), nrhs,
        work.active(work_len),
        x_vec.active(cols * nrhs));
}

/**
 * @}
 */

} // namespace fsb
