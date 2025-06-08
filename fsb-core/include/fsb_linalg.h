#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * @defgroup LinearAlgebra Linear Algebra
 * @brief Linear algebra utility for matrix and vector computations
 *
 * @{
 */

#define FSB_MAX(a, b) (((a) > (b)) ? (a) : (b))

/**
 * @brief Floating point type
 */
typedef double double_t;

/**
 * @brief Error codes for linear algebra functions
 */
typedef enum FsbLapackErrorType
{
    /**
     * @brief No error
     */
    EFSB_LAPACK_ERROR_NONE = 0,
    /**
     * @brief Input value error
     */
    EFSB_LAPACK_ERROR_INPUT = 1,
    /**
     * @brief Not enough memory
     */
    EFSB_LAPACK_ERROR_MEMORY = 2,
    /**
     * @brief Solution did not converge
     */
    EFSB_LAPACK_ERROR_CONVERGE = 3,
    /**
     * @brief Work query failed
     */
    EFSB_LAPACK_ERROR_QUERY = 4,
    /**
     * @brief Matrix not positive definite
     */
    EFSB_LAPACK_NOT_POSITIVE_DEFINITE = 5,
    /**
     * @brief Input matrix is singular
     */
    EFSB_LAPACK_SINGULAR = 6
} FsbLinalgErrorType;

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
FsbLinalgErrorType fsb_linalg_svd(
    const double_t mat[], size_t rows, size_t cols, bool u_full, bool v_full, size_t work_len, double_t work[], double_t unitary_u[],
    double_t sing_val[], double_t unitary_vt[]);

/**
 * @brief Eigenvalue decomposition
 *
 * @param[in]  mat          input matrix (sxs)
 * @param[in]  dim          matrix A dimension
 * @param[in]  work_len    buffer
 * @param[in,out]  work    buffer
 * @param[out] val_real  real components of the eigenvalues (sx1)
 * @param[out] val_imag  imaginary components of the eigenvalues (sx1)
 * @param[out] vec_real  real components of the eigenvectors (sxs)
 * @param[out] vec_imag  imaginary components of the eigenvectors (sxs)
 *
 * @return     error code
 */
FsbLinalgErrorType fsb_linalg_matrix_eig(
    const double_t mat[], size_t dim, size_t work_len, double_t work[],
    double_t val_real[], double_t val_imag[],
    double_t vec_real[], double_t vec_imag[]);

/**
 * @brief Eigenvalue decomposition for symmetric lower triangular matrix
 *
 * @param[in]    mat   input matrix (sxs)
 * @param[in]    dim   matrix A dimension
 * @param[in]  work_len    buffer
 * @param[in,out]  work    buffer
 * @param[out]   val eigenvalues (sx1)
 * @param[out]   vec column eigenvectors (sxs)
 *
 */
FsbLinalgErrorType fsb_linalg_sym_lt_eig(
    const double_t mat[], size_t dim, size_t work_len, double_t work[],
    double_t val[], double_t vec[]);

/**
 * @brief Cholesky factorization for symmetric positive definite matrix
 *
 * @param mat Input matrix (dim x dim)
 * @param dim Matrix A dimension s
 * @param mat_chol Cholesky factorization of input matrix A (dim x dim)
 * @return Error code
 */
FsbLinalgErrorType fsb_linalg_cholesky_decomposition(
    const double_t mat[], size_t dim, double_t mat_chol[]);

/**
 * @brief Cholesky solve
 *
 * square symmetric positive-definite matrix cholesky solve.
 * solve for b from x = A b
 * Variable x is overwritten by b on exit
 *
 * @param mat
 * @param b_vec
 * @param nrhs
 * @param dim
 * @param work_len
 * @param work
 * @param x_vec
 * @return
 */
FsbLinalgErrorType fsb_linalg_cholesky_solve(const double_t mat[], const double_t b_vec[],
                                             size_t nrhs, size_t dim,
                                             size_t work_len, double_t work[],
                                             double_t x_vec[]);

/**
 * @brief Check if lower triangular symmetric matrix is positive definite
 *
 * @param mat Input lower triangular matrix (sxs)
 * @param dim Matrix A dimension
 * @param work_len Work vector length
 * @param work Work vector
 * @return true
 * @return false
 */
bool fsb_linalg_is_posdef(const double_t mat[], size_t dim, size_t work_len, double_t work[]);

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
FsbLinalgErrorType fsb_linalg_matrix_sqr_solve(
    const double_t mat[], const double_t y_vec[], size_t nrhs, size_t dim, size_t work_len,
    size_t iwork_len, double_t work[], int iwork[], double_t x_vec[]);

/**
 * @brief Inverse of a matrix where number of columns are great er than or equal to number of rows
 *
 * Work length must be at least rows * MIN(rows, columns) + MIN(rows, columns) + MIN(rows, columns) * columns
 * and additional length required by the fsb_linalg_svd function.
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
FsbLinalgErrorType fsb_linalg_pseudoinverse(const double_t mat[], size_t rows, size_t columns, size_t work_len, double_t work[], double_t inv_mat[]);

/**
 * @brief Solve overdetermined or underdetermined system using least squares
 *
 * Solves min ||A*X - B|| for X where A is an M-by-N matrix
 * using the QR or LQ factorization of A.
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
FsbLinalgErrorType fsb_linalg_leastsquares_solve(
    const double_t mat[], size_t rows, size_t columns, const double_t b_vec[], size_t nrhs,
    size_t work_len, double_t work[], double_t x_vec[]);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif
