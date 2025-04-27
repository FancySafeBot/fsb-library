
#include <stdint.h>
#include <string.h>

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
        for (size_t k = 0U; k < a_len; ++k)
        {
            a_mat_tmp[k] = mat[k];
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
        for (size_t k = 0U; k < a_len; ++k)
        {
            a_mat_tmp[k] = mat[k];
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
                size_t k = 0U;
                while (k < dim)
                {
                    if (val_imag[k] > 0.0)
                    {
                        /* increment */
                        k++;
                        if (k < dim)
                        {
                            /* complex conjugate pairs */
                            for (size_t j = 0U; j < dim; ++j)
                            {
                                vec_imag[(k - 1U) * dim + j] = vec_real[k * dim + j];
                                vec_imag[k * dim + j] = -vec_real[k * dim + j];
                                vec_real[k * dim + j] = vec_real[(k - 1U) * dim + j];
                            }
                        }
                    }
                    else
                    {
                        /* no imaginary vector */
                        for (size_t j = 0U; j < dim; ++j)
                        {
                            vec_imag[k * dim + j] = 0.0;
                        }
                    }
                    /* increment */
                    k++;
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
        for (size_t k = 0U; k < a_len; ++k)
        {
            vec[k] = mat[k];
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
        for (size_t k = 0U; k < a_len; ++k)
        {
            mat_chol[k] = mat[k];
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
        for (size_t k = 0U; k < a_len; ++k)
        {
            mat_temp[k] = mat[k];
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
            for (size_t k = 0U; k < (nrhs * dim); ++k)
            {
                x_vec[k] = b_vec[k];
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
