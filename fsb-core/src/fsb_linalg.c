#include <stdint.h>
#include <string.h>
#include <printf.h>

#include "fsb_linalg.h"
#include "openblas/lapack.h"

FsbLinalgErrorType fsb_linalg_svd(
               const double_t mat[], const size_t rows, const size_t cols,
               const bool u_full, const bool v_full,
               const size_t work_len, double_t work[],
               double_t unitary_u[], double_t sing_val[], double_t unitary_vt[])
{
    FsbLinalgErrorType retval = EFSB_LAPACK_ERROR_NONE;

    if (cols >= (INT32_MAX / rows))
    {
        retval = EFSB_LAPACK_ERROR_INPUT;
    }
    else if (work_len < (rows * cols))
    {
        // not enough buffer for copy of A
        retval = EFSB_LAPACK_ERROR_MEMORY;
    }
    else
    {
        const size_t a_len = rows * cols;
        double_t* a_mat_tmp = work;
        const size_t work_partial_len = a_len;

        const char* u_opt = (u_full ? "A" : "S");
        const char* v_opt = (v_full ? "A" : "S");
        const size_t u_opt_size = 1U;
        const size_t v_opt_size = 1U;

        // copy A to buffer
        for (size_t idx = 0U; idx < a_len; ++idx)
        {
            a_mat_tmp[idx] = mat[idx];
        }

        const lapack_int l_m = (lapack_int)rows;
        const lapack_int l_n = (lapack_int)cols;
        const lapack_int ldvt = (lapack_int)(((rows < cols) && !v_full) ? rows : cols);
        double_t work_query = 0.0;
        lapack_int lwork = -1;
        lapack_int info = 0;
        dgesvd_(u_opt, v_opt, &l_m, &l_n, a_mat_tmp, &l_m, sing_val, unitary_u, &l_m, unitary_vt, &ldvt, &work_query, &lwork, &info, u_opt_size, v_opt_size);
        lwork = (lapack_int)(work_query);
        const size_t s_lwork = (size_t)lwork;

        if (info != 0)
        {
            /* dgesvd work query failed */
            retval = EFSB_LAPACK_ERROR_QUERY;
        }
        else if (work_len < (work_partial_len + s_lwork))
        {
            // not enough buffer for svd operation
            retval = EFSB_LAPACK_ERROR_MEMORY;
        }
        else
        {
            // allocate work
            double_t* svd_work = &work[work_partial_len];

            /* perform SVD */
            dgesvd_(u_opt, v_opt, &l_m, &l_n, a_mat_tmp, &l_m, sing_val, unitary_u, &l_m, unitary_vt, &ldvt, svd_work, &lwork, &info, u_opt_size, v_opt_size);
            if (info < 0)
            {
                // Some intput error
                retval = EFSB_LAPACK_ERROR_INPUT;
            }
            else if (info > 0)
            {
                // DBDSQR did not converge (see DGESVD)
                retval = EFSB_LAPACK_ERROR_CONVERGE;
            }
            else
            {
                // success
            }
        }
    }
    return retval;
}

FsbLinalgErrorType fsb_linalg_matrix_eig(
        const double_t mat[], const size_t dim, const size_t work_len, double_t work[],
        double_t val_real[], double_t val_imag[],
        double_t vec_real[], double_t vec_imag[])
{
    FsbLinalgErrorType retval = EFSB_LAPACK_ERROR_NONE;
    /* alias data */
    if (dim >= (INT32_MAX / dim))
    {
        retval = EFSB_LAPACK_ERROR_INPUT;
    }
    else if (work_len < (dim * dim))
    {
        // not enough buffer for copy of A
        retval = EFSB_LAPACK_ERROR_MEMORY;
    }
    else
    {
        const size_t a_len = dim * dim;
        double_t* a_mat_tmp = work;
        const size_t work_partial_len = a_len;

        // copy A to buffer
        for (size_t idx = 0U; idx < a_len; ++idx)
        {
            a_mat_tmp[idx] = mat[idx];
        }

        // work query inputs
        const char* left_vec_opt = "N";
        const char* right_vec_opt = "V";
        const size_t left_opt_size = 1U;
        const size_t right_opt_size = 1U;
        const lapack_int l_s = (lapack_int)dim;
        const lapack_int lvd = 1;
        double_t work_query = 0.0;
        lapack_int lwork = -1;
        lapack_int info = 0;

        // query work size
        dgeev_(left_vec_opt, right_vec_opt,
               &l_s, a_mat_tmp, &l_s,
               val_real, val_imag,
               NULL, &lvd,
               vec_real, &l_s,
               &work_query, &lwork, &info,
               left_opt_size, right_opt_size);
        lwork = (lapack_int)(work_query);
        const size_t s_lwork = (size_t)lwork;

        if (info != 0)
        {
            /* dgeev_ work query failed */
            retval = EFSB_LAPACK_ERROR_QUERY;
        }
        else if (work_len < (work_partial_len + s_lwork))
        {
            // not enough buffer for operation
            retval = EFSB_LAPACK_ERROR_MEMORY;
        }
        else
        {
            // allocate lwork
            double_t* eig_work = &work[work_partial_len];

            /* perform decomposition */
            dgeev_(left_vec_opt, right_vec_opt,
                &l_s, a_mat_tmp, &l_s,
                val_real, val_imag,
                NULL, &lvd,
                vec_real, &l_s,
                eig_work, &lwork, &info,
                left_opt_size, right_opt_size);

            if (info < 0)
            {
                // Some intput error
                retval = EFSB_LAPACK_ERROR_INPUT;
            }
            else if (info > 0)
            {
                // DBDSQR did not converge (see DGESVD)
                retval = EFSB_LAPACK_ERROR_CONVERGE;
            }
            else
            {
                /* populate val_imag */
                /* initialize */
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

FsbLinalgErrorType fsb_linalg_sym_lt_eig(
    const double_t mat[], const size_t dim, const size_t work_len, double_t work[],
    double_t val[], double_t vec[])
{

    FsbLinalgErrorType retval = EFSB_LAPACK_ERROR_NONE;
    /* alias data */
    if (dim >= (INT32_MAX / dim))
    {
        retval = EFSB_LAPACK_ERROR_INPUT;
    }
    else
    {
        const size_t a_len = dim * dim;

        // copy A to output vector
        for (size_t idx = 0U; idx < a_len; ++idx)
        {
            vec[idx] = mat[idx];
        }

        // work query inputs
        const char* job_opt = "V";
        const char* uplo_opt = "L";
        const size_t job_opt_size = 1U;
        const size_t uplo_opt_size = 1U;
        const lapack_int l_s = (lapack_int)dim;
        double_t work_query = 0.0;
        lapack_int lwork = -1;
        lapack_int info = 0;

        // query work size
        dsyev_(job_opt, uplo_opt,
               &l_s, vec, &l_s,
               val,
               &work_query, &lwork, &info,
               job_opt_size, uplo_opt_size);
        lwork = (lapack_int)(work_query);
        const size_t s_lwork = (size_t)lwork;

        if (info != 0)
        {
            /* dgeev_ work query failed */
            retval = EFSB_LAPACK_ERROR_QUERY;
        }
        else if (work_len < s_lwork)
        {
            // not enough buffer for operation
            retval = EFSB_LAPACK_ERROR_MEMORY;
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
                // Intput error
                retval = EFSB_LAPACK_ERROR_INPUT;
            }
            else if (info > 0)
            {
                // did not converge (see dsyev)
                retval = EFSB_LAPACK_ERROR_CONVERGE;
            }
            else
            {
                // success
            }
        }
    }

    return retval;
}

FsbLinalgErrorType fsb_linalg_cholesky_decomposition(
    const double_t mat[], const size_t dim, double_t mat_chol[])
{
    FsbLinalgErrorType retval = EFSB_LAPACK_ERROR_NONE;
    /* alias data */
    if (dim >= (INT32_MAX / dim))
    {
        retval = EFSB_LAPACK_ERROR_INPUT;
    }
    else
    {
        const size_t a_len = dim * dim;
        const char* uplo_opt = "L";
        const size_t uplo_opt_size = 1U;
        const lapack_int l_s = (lapack_int)dim;
        lapack_int info = 0;

        // copy A to output vector
        for (size_t idx = 0U; idx < a_len; ++idx)
        {
            mat_chol[idx] = mat[idx];
        }

        /* cholesky factorization */
        const lapack_int chol_ret = dpotrf_(uplo_opt, &l_s, mat_chol, &l_s, &info, uplo_opt_size);
        (void)chol_ret;

        if (info > 0)
        {
            retval = EFSB_LAPACK_NOT_POSITIVE_DEFINITE;
        }
        else if (info < 0)
        {
            retval = EFSB_LAPACK_ERROR_INPUT;
        }
        else
        {
            // no error
        }
    }
    return retval;
}

FsbLinalgErrorType fsb_linalg_cholesky_solve(const double_t mat[], const double_t b_vec[],
                                             const size_t nrhs, const size_t dim,
                                             const size_t work_len, double_t work[],
                                             double_t x_vec[])
{
    FsbLinalgErrorType retval = EFSB_LAPACK_ERROR_NONE;
    /* alias data */
    if (dim >= (INT32_MAX / dim))
    {
        retval = EFSB_LAPACK_ERROR_INPUT;
    }
    else if (work_len < (dim * dim))
    {
        // not enough buffer for copy of A
        retval = EFSB_LAPACK_ERROR_MEMORY;
    }
    else
    {
        double* mat_temp = work;
        const size_t a_len = dim * dim;
        // copy mat to buffer
        for (size_t idx = 0U; idx < a_len; ++idx)
        {
            mat_temp[idx] = mat[idx];
        }

        const char* uplo_opt = "L";
        const size_t uplo_opt_size = 1U;
        const lapack_int l_s = (lapack_int)dim;
        const lapack_int l_nrhs = (lapack_int)nrhs;
        lapack_int info = 0;

        /* DPOTRF( UPLO, N, A, LDA, INFO ) */
        /* cholesky factorization */
        const lapack_int dpotrf_ret = dpotrf_(uplo_opt, &l_s, mat_temp, &l_s, &info, uplo_opt_size);
        (void)dpotrf_ret;

        if (info == 0)
        {
            // no error, copy b to x
            for (size_t idx = 0U; idx < (nrhs * dim); ++idx)
            {
                x_vec[idx] = b_vec[idx];
            }
            dpotrs_(uplo_opt, &l_s, &l_nrhs, mat_temp, &l_s, x_vec, &l_s, &info, uplo_opt_size);
        }

        if (info > 0)
        {
            retval = EFSB_LAPACK_NOT_POSITIVE_DEFINITE;
        }
        else if (info < 0)
        {
            retval = EFSB_LAPACK_ERROR_INPUT;
        }
        else
        {
            /* no error */
        }
    }

    return retval;
}

bool fsb_linalg_is_posdef(const double_t mat[], const size_t dim, const size_t work_len, double_t work[])
{
    bool retval = false;
    if (dim < (INT32_MAX / dim))
    {
        const size_t a_len = dim * dim;
        if (work_len >= a_len)
        {
            // attempt cholesky factorization
            const FsbLinalgErrorType err = fsb_linalg_cholesky_decomposition(mat, dim, work);
            retval = (err == EFSB_LAPACK_ERROR_NONE);
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
FsbLinalgErrorType fsb_linalg_matrix_sqr_solve(
    const double_t mat[], const double_t y_vec[], const size_t nrhs, const size_t dim, const size_t work_len,
    const size_t iwork_len, double_t work[], lapack_int iwork[], double_t x_vec[])
{
    FsbLinalgErrorType retval = EFSB_LAPACK_ERROR_NONE;
    /* alias data */
    if (dim >= (INT32_MAX / dim))
    {
        retval = EFSB_LAPACK_ERROR_INPUT;
    }
    else if (work_len < (dim * dim) || (iwork_len < dim))
    {
        // not enough buffer for copy of A or y
        retval = EFSB_LAPACK_ERROR_MEMORY;
    }
    else
    {
        double* mat_lu = work;
        lapack_int* ipiv = iwork;
        const size_t a_len = dim * dim;
        const lapack_int l_s = (lapack_int)dim;
        // copy mat to buffer
        for (size_t idx = 0U; idx < a_len; ++idx)
        {
            mat_lu[idx] = mat[idx];
        }

        /* DGETRF( M, N, A, LDA, IPIV, INFO ) */
        lapack_int info = 0;
        lapack_int dgetrf_ret = dgetrf_(&l_s, &l_s, mat_lu, &l_s, ipiv, &info);
        (void)dgetrf_ret;

        if (info == 0)
        {
            const char*      transpose_opt = "N";
            const size_t     transpose_opt_size = 1U;
            const lapack_int l_nrhs = (lapack_int)nrhs;
            // copy y to x vector
            for (size_t idx = 0U; idx < dim; ++idx)
            {
                x_vec[idx] = y_vec[idx];
            }
            //      SUBROUTINE DGETRS( TRANS, N, NRHS, A, LDA, IPIV, B, LDB, INFO )
            lapack_int dgetrs_ret = dgetrs_(transpose_opt, &l_s, &l_nrhs, mat_lu, &l_s, ipiv, x_vec, &l_s, &info, transpose_opt_size);
            (void)dgetrs_ret;
        }

        if (info > 0)
        {
            retval = EFSB_LAPACK_SINGULAR;
            // if INFO = i, U(i,i) is exactly zero. The factorization
            //     has been completed, but the factor U is exactly
            //     singular, and division by zero will occur if it is used
            //     to solve a system of equations.
        }
        else if (info < 0)
        {
            retval = EFSB_LAPACK_ERROR_INPUT;
        }
        else
        {
            /* no error */
        }
    }

    return retval;
}

FsbLinalgErrorType fsb_linalg_pseudoinverse(
    const double_t mat[], const size_t rows, const size_t columns, const size_t work_len, double_t work[], double inv_mat[])
{
    FsbLinalgErrorType retval = EFSB_LAPACK_ERROR_NONE;

    if (rows >= (INT32_MAX / columns))
    {
        return EFSB_LAPACK_ERROR_INPUT;
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
        return EFSB_LAPACK_ERROR_MEMORY;
    }

    // Partition the work buffer
    double_t* u_mat = work;
    double_t* s_vec = &work[size_u];
    double_t* vt_mat = &work[size_u + size_s];

    // Use remaining workspace for SVD computation
    const size_t svd_work_len = work_len - size_usvt_work;
    double_t* svd_work = &work[size_usvt_work];

    // Perform SVD
    retval = fsb_linalg_svd(mat, rows, columns, false, false, svd_work_len, svd_work,
                           u_mat, s_vec, vt_mat);
    if (retval != EFSB_LAPACK_ERROR_NONE)
    {
        return retval;
    }

    // Compute pseudoinverse: V * S^+ * U^T
    // First, create S^+ by inverting non-zero singular values
    const double_t epsilon = 1.0e-12;  // Small threshold for numerical stability

    // Initialize the output inverse matrix with zeros
    memset(inv_mat, 0, columns * rows * sizeof(double_t));

    // Compute V * S^+
    for (size_t jdx = 0; jdx < columns; ++jdx)
    {
        for (size_t idx = 0; idx < min_dim; ++idx)
        {
            if (s_vec[idx] > epsilon)
            {
                // Scale the corresponding row in V^T by 1/s_i
                for (size_t kdx = 0; kdx < min_dim; ++kdx)
                {
                    // v_ji = v_ji / s_i where v_j is column j of V
                    double_t v_element = vt_mat[kdx * columns + jdx];
                    inv_mat[jdx * rows + idx] += (v_element / s_vec[kdx]);
                }
            }
            // If singular value is effectively zero, contribution remains zero
        }
    }

    // Multiply by U^T to get final result: (V * S^+) * U^T
    for (size_t idx = 0; idx < columns; ++idx)
    {
        for (size_t jdx = 0; jdx < rows; ++jdx)
        {
            double_t sum = 0.0;
            for (size_t kdx = 0; kdx < min_dim; ++kdx)
            {
                sum += inv_mat[idx * rows + kdx] * u_mat[jdx * min_dim + kdx];
            }
            inv_mat[idx * rows + jdx] = sum;
        }
    }

    return retval;
}

FsbLinalgErrorType fsb_linalg_leastsquares_solve(
    const double_t mat[], const size_t rows, const size_t columns, const double_t b_vec[], const size_t nrhs,
    size_t work_len, double_t work[], double_t x_vec[])
{
    FsbLinalgErrorType retval = EFSB_LAPACK_ERROR_NONE;

    if (rows == 0 || columns == 0 || nrhs == 0 || work_len == 0 ||
        (rows >= (INT32_MAX / columns) || (nrhs >= INT32_MAX / columns) || (nrhs >= INT32_MAX / rows)))
    {
        // Input dimensions are too large
        retval = EFSB_LAPACK_ERROR_INPUT;
    }
    else if (work_len < ((rows * columns) + (FSB_MAX(rows, columns) * nrhs)))
    {
        retval = EFSB_LAPACK_ERROR_MEMORY;
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
        double_t* a_copy = work;
        double_t* b_work = &work[size_a];
        // Copy matrix A to work buffer
        for (size_t idx = 0; idx < size_a; ++idx)
        {
            a_copy[idx] = mat[idx];
        }
        // Copy b_vec to b_work
        for (size_t col = 0; col < nrhs; ++col)
        {
            for (size_t row = 0; row < rows; ++row)
            {
                b_work[col * size_ldb + row] = b_vec[col * rows + row];
            }
        }

        // Set up parameters for dgels
        const char* trans_opt = "N";  // No transpose
        const size_t trans_opt_size = 1U;
        const lapack_int l_rows = (lapack_int)rows;
        const lapack_int l_cols = (lapack_int)columns;
        const lapack_int l_nrhs = (lapack_int)nrhs;
        const lapack_int lda = l_rows;
        const lapack_int ldb = (lapack_int)size_ldb;  // leading dimension of b
        lapack_int info = 0;

        // Query workspace size
        double_t work_query = 0.0;
        lapack_int lwork = -1;
        dgels_(trans_opt, &l_rows, &l_cols, &l_nrhs,
               a_copy, &lda, b_work, &ldb,
               &work_query, &lwork, &info,
               trans_opt_size);
        if (info != 0)
        {
            retval = EFSB_LAPACK_ERROR_QUERY;
        }
        else
        {
            lwork = (lapack_int)(work_query);
            const size_t s_lwork = (size_t)lwork;
            // Check if we have enough workspace
            if (work_len < (work_len_ab + s_lwork))
            {
                retval = EFSB_LAPACK_ERROR_MEMORY;
            }
        }

        // Solve with dgels
        if (retval == EFSB_LAPACK_ERROR_NONE)
        {
            // Allocate workspace for dgels
            double_t* dgels_work = &work[work_len_ab];
            // Solve least squares problem using dgels
            dgels_(trans_opt, &l_rows, &l_cols, &l_nrhs,
                   a_copy, &lda, b_work, &ldb,
                   dgels_work, &lwork, &info,
                   trans_opt_size);
            if (info < 0)
            {
                retval = EFSB_LAPACK_ERROR_INPUT;
            }
            else if (info > 0)
            {
                retval = EFSB_LAPACK_ERROR_NOT_FULL_RANK;
            }
        }

        if (retval == EFSB_LAPACK_ERROR_NONE)
        {
            // b_work to x_vec conversion
            const size_t output_rows = columns;
            for (size_t col = 0; col < nrhs; ++col)
            {
                for (size_t row = 0; row < output_rows; ++row)
                {
                    x_vec[col * output_rows + row] = b_work[col * size_ldb + row];
                }
            }
        }
    }
    return retval;
}
