#pragma once

#include <cmath>
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
enum class LinalgErrorType : uint8_t
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
 * @brief Cholesky factorization for a symmetric positive definite matrix
 *
 * Computes the lower triangular Cholesky factorization A = L * L^T of a
 * symmetric positive definite matrix stored in lower triangular form.
 *
 * @param[in]  mat      Input symmetric positive definite matrix (dim x dim)
 * @param[in]  dim      Matrix dimension
 * @param[out] mat_chol Lower triangular Cholesky factor L (dim x dim)
 * @return Error code; @c NOT_POSITIVE_DEFINITE if the matrix is not positive definite
 */
LinalgErrorType linalg_cholesky_decomposition(
    Span<const Real> mat, size_t dim,
    Span<Real> mat_chol);

/**
 * @brief Cholesky solve
 *
 * Solves the symmetric positive-definite system A * x = b using
 * Cholesky factorization (DPOTRF / DPOTRS). @p work must hold at least
 * dim × dim elements for the factorization buffer.
 *
 * @param[in]    mat   Input symmetric positive-definite matrix (dim x dim)
 * @param[in]    dim   Matrix dimension
 * @param[in]    b_vec Right-hand side matrix (dim x nrhs)
 * @param[in]    nrhs  Number of right-hand side vectors
 * @param[in,out] work  Work buffer (at least dim × dim elements)
 * @param[out]   x_vec Solution matrix (dim x nrhs)
 * @return Error code; @c NOT_POSITIVE_DEFINITE if the matrix is not positive definite
 */
LinalgErrorType linalg_cholesky_solve(
    Span<const Real> mat, size_t dim,
    Span<const Real> b_vec, size_t nrhs,
    Span<Real> work,
    Span<Real> x_vec);

/**
 * @brief Check if a symmetric matrix (lower triangular storage) is positive definite
 *
 * Attempts a Cholesky factorization. @p work must hold at least dim × dim elements.
 *
 * @param[in]    mat  Input lower triangular symmetric matrix (dim x dim)
 * @param[in]    dim  Matrix dimension
 * @param[in,out] work Work buffer (at least dim × dim elements)
 * @return @c true if the matrix is positive definite, @c false otherwise
 */
bool linalg_is_posdef(
    Span<const Real> mat, size_t dim,
    Span<Real> work);

/**
 * @brief Solve a square linear system A * x = b using LU factorization
 *
 * Uses DGETRF (LU with partial pivoting) followed by DGETRS.
 * @p work must hold at least dim × dim elements; @p iwork must hold at least dim elements.
 *
 * @param[in]    mat   Input square matrix (dim x dim)
 * @param[in]    dim   Matrix dimension
 * @param[in]    y_vec Right-hand side matrix (dim x nrhs)
 * @param[in]    nrhs  Number of right-hand side vectors
 * @param[in,out] work  Real work buffer (at least dim × dim elements)
 * @param[in,out] iwork Integer pivot buffer (at least dim elements)
 * @param[out]   x_vec Solution matrix (dim x nrhs)
 * @return Error code; @c SINGULAR if the matrix is exactly singular
 */
LinalgErrorType linalg_matrix_sqr_solve(
    Span<const Real> mat, size_t dim,
    Span<const Real> y_vec, size_t nrhs,
    Span<Real> work,
    Span<lapack_int> iwork,
    Span<Real> x_vec);

/**
 * @brief Moore-Penrose pseudoinverse via SVD
 *
 * Computes the pseudoinverse of an arbitrary (rows × cols) matrix using
 * thin SVD. @p work must hold at least
 * rows × min(rows,cols) + min(rows,cols) + min(rows,cols) × cols elements,
 * plus the additional workspace required by linalg_svd().
 *
 * @param[in]    mat     Input matrix (rows x cols)
 * @param[in]    rows    Number of rows
 * @param[in]    cols    Number of columns
 * @param[in,out] work    Work buffer (see size requirement above)
 * @param[out]   inv_mat Pseudoinverse matrix (cols x rows)
 * @return Error code
 */
LinalgErrorType linalg_pseudoinverse(
    Span<const Real> mat, size_t rows, size_t cols,
    Span<Real> work,
    Span<Real> inv_mat);

/**
 * @brief Solve an overdetermined or underdetermined least-squares problem
 *
 * Solves min‖A·X − B‖₂ for X using DGELS (QR or LQ factorization).
 * @p work must hold at least rows × cols + max(rows,cols) × nrhs elements,
 * plus the additional LAPACK workspace returned by the internal work query.
 *
 * @param[in]    mat   Input matrix A (rows x cols)
 * @param[in]    rows  Number of rows in A
 * @param[in]    cols  Number of columns in A
 * @param[in]    b_vec Right-hand side matrix B (rows x nrhs)
 * @param[in]    nrhs  Number of right-hand side vectors
 * @param[in,out] work  Work buffer (see size requirement above)
 * @param[out]   x_vec Solution matrix X (cols x nrhs)
 * @return Error code; @c ERROR_NOT_FULL_RANK if A is rank-deficient
 */
LinalgErrorType linalg_leastsquares_solve(
    Span<const Real> mat, size_t rows, size_t cols,
    Span<const Real> b_vec, size_t nrhs,
    Span<Real> work,
    Span<Real> x_vec);

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
 * @param[in]     mat        Input matrix to decompose (rows x cols)
 * @param[in]     rows       Number of rows
 * @param[in]     cols       Number of columns
 * @param[in]     u_full     Compute full U matrix (true) or thin U (false)
 * @param[in]     v_full     Compute full V^T matrix (true) or thin V^T (false)
 * @param[in,out] work       Work buffer; must hold at least rows × cols elements
 *                           plus the LAPACK workspace returned by the internal work query
 * @param[out]    unitary_u  Unitary matrix U
 * @param[out]    sing_val   Singular values (s x 1)
 * @param[out]    unitary_vt Transpose of unitary matrix V^T
 */
LinalgErrorType linalg_svd(
    Span<const Real> mat, size_t rows, size_t cols,
    bool u_full, bool v_full,
    Span<Real> work,
    Span<Real> unitary_u,
    Span<Real> sing_val,
    Span<Real> unitary_vt);

/**
 * @brief Eigenvalue decomposition for a general square matrix
 *
 * Computes all eigenvalues and optionally right eigenvectors of a general
 * dim x dim matrix using DGEEV.
 *
 * @param[in]    mat       Input square matrix (dim x dim)
 * @param[in]    dim       Matrix dimension
 * @param[in,out] work     Work buffer; must hold at least dim × dim elements
 *                         plus the LAPACK workspace returned by the internal work query
 * @param[out]   val_real  Real parts of eigenvalues (dim x 1)
 * @param[out]   val_imag  Imaginary parts of eigenvalues (dim x 1)
 * @param[out]   vec_real  Real parts of right eigenvectors, column-major (dim x dim)
 * @param[out]   vec_imag  Imaginary parts of right eigenvectors, column-major (dim x dim)
 * @return Error code
 */
LinalgErrorType linalg_matrix_eig(
    Span<const Real> mat, size_t dim,
    Span<Real> work,
    Span<Real> val_real,
    Span<Real> val_imag,
    Span<Real> vec_real,
    Span<Real> vec_imag);

/**
 * @brief Eigenvalue decomposition for a real symmetric matrix (lower triangular storage)
 *
 * Computes all eigenvalues and eigenvectors of a real symmetric matrix stored
 * in lower triangular form. On exit, @p vec contains the orthonormal eigenvectors
 * as columns and @p val contains the corresponding eigenvalues in ascending order.
 *
 * @param[in]     mat   Input symmetric matrix in lower triangular form (dim x dim)
 * @param[in]     dim   Matrix dimension
 * @param[in,out] work  Work buffer; must be at least as large as the workspace
 *                      size returned by the internal LAPACK work query
 * @param[out]    val   Eigenvalues in ascending order (dim x 1)
 * @param[out]    vec   Column eigenvectors (dim x dim)
 * @return Error code
 */
LinalgErrorType linalg_sym_lt_eig(
    Span<const Real> mat, size_t dim,
    Span<Real> work,
    Span<Real> val,
    Span<Real> vec);

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
    return linalg_sym_lt_eig(
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
    return linalg_cholesky_decomposition(
        mat.active(dim * dim), dim,
        mat_chol.active(dim * dim));
}

template <size_t MaxMat, size_t MaxWorkLen>
inline bool linalg_is_posdef_array(
    const Array<MaxMat>& mat, size_t dim,
    Array<MaxWorkLen>& work, size_t work_len)
{
    return linalg_is_posdef(mat.active(dim * dim), dim, work.active(work_len));
}

template <size_t MaxMat, size_t MaxVec, size_t MaxWorkLen>
inline LinalgErrorType linalg_cholesky_solve_array(
    const Array<MaxMat>& mat, size_t dim,
    const Array<MaxVec>& b_vec, size_t nrhs,
    Array<MaxWorkLen>& work, size_t work_len,
    Array<MaxVec>& x_vec)
{
    return linalg_cholesky_solve(
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
    return linalg_matrix_sqr_solve(
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
    return linalg_pseudoinverse(
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
    return linalg_leastsquares_solve(
        mat.active(rows * cols), rows, cols,
        b_vec.active(rows * nrhs), nrhs,
        work.active(work_len),
        x_vec.active(cols * nrhs));
}

/**
 * @}
 */

} // namespace fsb
