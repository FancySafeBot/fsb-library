#include <algorithm>
#include <cstdint>
#include <cstring>

#include "fsb_linalg.h"

#include "openblas/lapack.h"

namespace fsb {

LinalgErrorType linalg_sym_lt_eig(
    const Real mat[], const size_t dim, const size_t work_len, Real work[],
    Real val[], Real vec[])
{

    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;
    /* alias data */
    if (dim == 0U)
    {
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else if (dim >= (static_cast<size_t>(INT32_MAX) / dim))
    {
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else
    {
        const size_t a_len = dim * dim;

        // copy A to output vector
        std::copy_n(mat, a_len, vec);

        // work query inputs
        const char* job_opt = "V";
        const char* uplo_opt = "L";
        const size_t job_opt_size = 1U;
        const size_t uplo_opt_size = 1U;
        const lapack_int l_s = static_cast<lapack_int>(dim);
        Real work_query = 0.0;
        lapack_int lwork = -1;
        lapack_int info = 0;

        // query work size
        dsyev_(job_opt, uplo_opt,
               &l_s, vec, &l_s,
               val,
               &work_query, &lwork, &info,
               job_opt_size, uplo_opt_size);
        lwork = static_cast<lapack_int>(work_query);
        const size_t s_lwork = static_cast<size_t>(lwork);

        if (info != 0)
        {
            /* dsyev_ work query failed */
            retval = LinalgErrorType::ERROR_QUERY;
        }
        else if (work_len < s_lwork)
        {
            // not enough buffer for operation
            retval = LinalgErrorType::ERROR_MEMORY;
        }
        else
        {
            // perform operation
            dsyev_(job_opt, uplo_opt,
                &l_s, vec, &l_s,
                val,
                work, &lwork, &info,
                job_opt_size, uplo_opt_size);

            if (info < 0)
            {
                // Input error
                retval = LinalgErrorType::ERROR_INPUT;
            }
            else if (info > 0)
            {
                // did not converge (see dsyev)
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

LinalgErrorType linalg_cholesky_decomposition(
    const Real mat[], const size_t dim, Real mat_chol[])
{
    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;
    /* alias data */
    if (dim == 0U)
    {
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else if (dim >= (static_cast<size_t>(INT32_MAX) / dim))
    {
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else
    {
        const size_t a_len = dim * dim;
        const char* uplo_opt = "L";
        const size_t uplo_opt_size = 1U;
        const lapack_int l_s = static_cast<lapack_int>(dim);
        lapack_int info = 0;

        // copy A to output vector
        std::copy_n(mat, a_len, mat_chol);

        /* cholesky factorization */
        const lapack_int chol_ret = dpotrf_(uplo_opt, &l_s, mat_chol, &l_s, &info, uplo_opt_size);
        (void)chol_ret;

        if (info > 0)
        {
            retval = LinalgErrorType::NOT_POSITIVE_DEFINITE;
        }
        else if (info < 0)
        {
            retval = LinalgErrorType::ERROR_INPUT;
        }
        else
        {
            // no error
        }
    }
    return retval;
}

LinalgErrorType linalg_cholesky_solve(const Real mat[], const Real b_vec[],
                                             const size_t nrhs, const size_t dim,
                                             const size_t work_len, Real work[],
                                             Real x_vec[])
{
    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;
    /* alias data */
    if (dim == 0U)
    {
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else if (dim >= (static_cast<size_t>(INT32_MAX) / dim))
    {
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else if (work_len < (dim * dim))
    {
        // not enough buffer for copy of A
        retval = LinalgErrorType::ERROR_MEMORY;
    }
    else
    {
        Real* mat_temp = work;
        const size_t a_len = dim * dim;
        // copy mat to buffer
        std::copy_n(mat, a_len, mat_temp);

        const char* uplo_opt = "L";
        const size_t uplo_opt_size = 1U;
        const lapack_int l_s = static_cast<lapack_int>(dim);
        const lapack_int l_nrhs = static_cast<lapack_int>(nrhs);
        lapack_int info = 0;

        /* DPOTRF( UPLO, N, A, LDA, INFO ) */
        /* cholesky factorization */
        const lapack_int dpotrf_ret = dpotrf_(uplo_opt, &l_s, mat_temp, &l_s, &info, uplo_opt_size);
        (void)dpotrf_ret;

        if (info == 0)
        {
            // no error, copy b to x
            std::copy_n(b_vec, (nrhs * dim), x_vec);
            dpotrs_(uplo_opt, &l_s, &l_nrhs, mat_temp, &l_s, x_vec, &l_s, &info, uplo_opt_size);
        }

        if (info > 0)
        {
            retval = LinalgErrorType::NOT_POSITIVE_DEFINITE;
        }
        else if (info < 0)
        {
            retval = LinalgErrorType::ERROR_INPUT;
        }
        else
        {
            /* no error */
        }
    }

    return retval;
}

bool linalg_is_posdef(const Real mat[], const size_t dim, const size_t work_len, Real work[])
{
    bool retval = false;
    if ((dim > 0U) && (dim < (static_cast<size_t>(INT32_MAX) / dim)))
    {
        const size_t a_len = dim * dim;
        if (work_len >= a_len)
        {
            // attempt cholesky factorization
            const LinalgErrorType err = linalg_cholesky_decomposition(mat, dim, work);
            retval = (err == LinalgErrorType::ERROR_NONE);
        }
    }
    return retval;
}

/* general LU solve m = n
 *
 *  DGETRF computes an LU factorization of a general M-by-N matrix A
 *  using partial pivoting with row interchanges.
 *
 *  The factorization has the form
 *     A = P * L * U
 */
LinalgErrorType linalg_matrix_sqr_solve(
    const Real mat[], const Real y_vec[], const size_t nrhs, const size_t dim, const size_t work_len,
    const size_t iwork_len, Real work[], lapack_int iwork[], Real x_vec[])
{
    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;
    /* alias data */
    if (dim == 0U)
    {
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else if (dim >= (static_cast<size_t>(INT32_MAX) / dim))
    {
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else if (work_len < (dim * dim) || (iwork_len < dim))
    {
        // not enough buffer for copy of A or y
        retval = LinalgErrorType::ERROR_MEMORY;
    }
    else
    {
        Real* mat_lu = work;
        lapack_int* ipiv = iwork;
        const size_t a_len = dim * dim;
        const lapack_int l_s = static_cast<lapack_int>(dim);
        // copy mat to buffer
        std::copy_n(mat, a_len, mat_lu);

        /* DGETRF( M, N, A, LDA, IPIV, INFO ) */
        lapack_int info = 0;
        const lapack_int dgetrf_ret = dgetrf_(&l_s, &l_s, mat_lu, &l_s, ipiv, &info);
        (void)dgetrf_ret;

        if (info == 0)
        {
            const char*      transpose_opt = "N";
            const size_t     transpose_opt_size = 1U;
            const lapack_int l_nrhs = static_cast<lapack_int>(nrhs);
            // copy y to x vector
            std::copy_n(y_vec, (nrhs * dim), x_vec);
            //      SUBROUTINE DGETRS( TRANS, N, NRHS, A, LDA, IPIV, B, LDB, INFO )
            const lapack_int dgetrs_ret = dgetrs_(transpose_opt, &l_s, &l_nrhs, mat_lu, &l_s, ipiv, x_vec, &l_s, &info, transpose_opt_size);
            (void)dgetrs_ret;
        }

        if (info > 0)
        {
            retval = LinalgErrorType::SINGULAR;
            // if INFO = i, U(i,i) is exactly zero. The factorization
            //     has been completed, but the factor U is exactly
            //     singular, and division by zero will occur if it is used
            //     to solve a system of equations.
        }
        else if (info < 0)
        {
            retval = LinalgErrorType::ERROR_INPUT;
        }
        else
        {
            /* no error */
        }
    }

    return retval;
}

LinalgErrorType linalg_pseudoinverse(
    const Real mat[], const size_t rows, const size_t columns, const size_t work_len, Real work[], Real inv_mat[])
{
    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;

    if ((rows == 0U) || (columns == 0U))
    {
        return LinalgErrorType::ERROR_INPUT;
    }
    if (rows >= (static_cast<size_t>(INT32_MAX) / columns))
    {
        return LinalgErrorType::ERROR_INPUT;
    }

    // Calculate the minimum dimension
    const size_t min_dim = (rows < columns) ? rows : columns;

    // Allocate working memory:
    // - For U matrix: rows × min_dim
    // - For S vector: min_dim
    // - For V^T matrix: min_dim × columns
    const size_t size_u = rows * min_dim;
    const size_t size_s = min_dim;
    const size_t size_vt = min_dim * columns;

    // Determine if we have enough work space
    const size_t size_usvt_work = size_u + size_s + size_vt;
    if (work_len < size_usvt_work)
    {
        return LinalgErrorType::ERROR_MEMORY;
    }

    // Partition the work buffer
    Real* u_mat = work;
    Real* s_vec = &work[size_u];
    Real* vt_mat = &work[size_u + size_s];

    // Use remaining workspace for SVD computation
    const size_t svd_work_len = work_len - size_usvt_work;
    Real* svd_work = &work[size_usvt_work];

    // Perform SVD
    retval = linalg_svd(
        Span<const Real>(mat, rows * columns), rows, columns,
        false, false,
        Span<Real>(svd_work, svd_work_len),
        Span<Real>(u_mat, size_u),
        Span<Real>(s_vec, size_s),
        Span<Real>(vt_mat, size_vt));
    if (retval != LinalgErrorType::ERROR_NONE)
    {
        return retval;
    }

    const Real epsilon = 1.0e-12;  // Small threshold for numerical stability

    // Initialize the output inverse matrix with zeros
    std::fill_n(inv_mat, columns * rows, 0.0);

    // Accumulate: inv_mat(i,j) += V(i,k) * (1/s_k) * U(j,k)
    // where inv_mat is (columns x rows).
    for (size_t k = 0U; k < min_dim; ++k)
    {
        const Real s = s_vec[k];
        if (s <= epsilon)
        {
            continue;
        }
        const Real inv_s = 1.0 / s;

        // V(i,k) can be read from V^T(k,i) stored in vt_mat (col-major): vt_mat[i*min_dim + k]
        for (size_t i = 0U; i < columns; ++i)
        {
            const Real v_ik = vt_mat[i * min_dim + k];
            const Real scale = v_ik * inv_s;

            // U(j,k) is u_mat[k*rows + j] (col-major)
            for (size_t j = 0U; j < rows; ++j)
            {
                inv_mat[j * columns + i] += scale * u_mat[k * rows + j];
            }
        }
    }

    return retval;
}

LinalgErrorType linalg_leastsquares_solve(
    const Real mat[], const size_t rows, const size_t columns, const Real b_vec[], const size_t nrhs,
    const size_t work_len, Real work[], Real x_vec[])
{
    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;

    if (rows == 0U || columns == 0U || nrhs == 0U || work_len == 0U ||
        (rows >= (static_cast<size_t>(INT32_MAX) / columns) || (nrhs >= static_cast<size_t>(INT32_MAX) / columns) || (nrhs >= static_cast<size_t>(INT32_MAX) / rows)))
    {
        // Input dimensions are too large
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else if (work_len < ((rows * columns) + (FSB_MAX(rows, columns) * nrhs)))
    {
        retval = LinalgErrorType::ERROR_MEMORY;
    }
    else
    {
        // Calculate required work space:
        // - For A matrix: rows * columns
        const size_t size_a = rows * columns;
        const size_t size_ldb = FSB_MAX(rows, columns);
        const size_t size_b_work = size_ldb * nrhs;
        const size_t work_len_ab = size_a + size_b_work;
        // Partition the work buffer for mat A
        Real* a_copy = work;
        Real* b_work = &work[size_a];
        // Copy matrix A to work buffer
        std::copy_n(mat, size_a, a_copy);
        // Copy b_vec to b_work
        for (size_t col = 0U; col < nrhs; ++col)
        {
            for (size_t row = 0U; row < rows; ++row)
            {
                b_work[col * size_ldb + row] = b_vec[col * rows + row];
            }
        }

        // Set up parameters for dgels
        const char* trans_opt = "N";  // No transpose
        const size_t trans_opt_size = 1U;
        const lapack_int l_rows = static_cast<lapack_int>(rows);
        const lapack_int l_cols = static_cast<lapack_int>(columns);
        const lapack_int l_nrhs = static_cast<lapack_int>(nrhs);
        const lapack_int lda = l_rows;
        const lapack_int ldb = static_cast<lapack_int>(size_ldb);  // leading dimension of b
        lapack_int info = 0;

        // Query workspace size
        Real work_query = 0.0;
        lapack_int lwork = -1;
        dgels_(trans_opt, &l_rows, &l_cols, &l_nrhs,
               a_copy, &lda, b_work, &ldb,
               &work_query, &lwork, &info,
               trans_opt_size);
        if (info != 0)
        {
            retval = LinalgErrorType::ERROR_QUERY;
        }
        else
        {
            lwork = static_cast<lapack_int>(work_query);
            const size_t s_lwork = static_cast<size_t>(lwork);
            // Check if we have enough workspace
            if (work_len < (work_len_ab + s_lwork))
            {
                retval = LinalgErrorType::ERROR_MEMORY;
            }
        }

        // Solve with dgels
        if (retval == LinalgErrorType::ERROR_NONE)
        {
            // Allocate workspace for dgels
            Real* dgels_work = &work[work_len_ab];
            // Solve least squares problem using dgels
            dgels_(trans_opt, &l_rows, &l_cols, &l_nrhs,
                   a_copy, &lda, b_work, &ldb,
                   dgels_work, &lwork, &info,
                   trans_opt_size);
            if (info < 0)
            {
                retval = LinalgErrorType::ERROR_INPUT;
            }
            else if (info > 0)
            {
                retval = LinalgErrorType::ERROR_NOT_FULL_RANK;
            }
        }

        if (retval == LinalgErrorType::ERROR_NONE)
        {
            // b_work to x_vec conversion
            const size_t output_rows = columns;
            for (size_t col = 0U; col < nrhs; ++col)
            {
                for (size_t row = 0U; row < output_rows; ++row)
                {
                    x_vec[col * output_rows + row] = b_work[col * size_ldb + row];
                }
            }
        }
    }
    return retval;
}

} // namespace fsb
