#include <algorithm>
#include <cstdint>
#include <cstring>

#include "fsb_linalg.h"

#include "openblas/lapack.h"

namespace fsb {

LinalgErrorType linalg_svd(
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
static void linalg_eig_populate_vec_imag(
    size_t dim,
    Span<Real> val_imag,
    Span<Real> vec_real,
    Span<Real> vec_imag)
{
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

LinalgErrorType linalg_matrix_eig(
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
        const auto s_lwork = static_cast<size_t>(lwork);

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
                linalg_eig_populate_vec_imag(dim, val_imag, vec_real, vec_imag);
            }
        }
    }
    return retval;
}

LinalgErrorType linalg_cholesky_decomposition(
    Span<const Real> mat, const size_t dim,
    Span<Real> mat_chol)
{
    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;
    /* alias data */
    if ((dim == 0U) || (dim >= (static_cast<size_t>(INT32_MAX) / dim)))
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
        std::copy_n(mat.data(), a_len, mat_chol.data());

        /* cholesky factorization */
        const lapack_int chol_ret = dpotrf_(uplo_opt, &l_s, mat_chol.data(), &l_s, &info, uplo_opt_size);
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

LinalgErrorType linalg_cholesky_solve(
    Span<const Real> mat, size_t dim,
    Span<const Real> b_vec, size_t nrhs,
    Span<Real> work,
    Span<Real> x_vec)
{
    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;
    /* alias data */
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
        Span<Real> mat_temp = work.active(a_len);
        // copy mat to buffer
        std::copy_n(mat.data(), a_len, mat_temp.data());

        const char* uplo_opt = "L";
        const size_t uplo_opt_size = 1U;
        const lapack_int l_s = static_cast<lapack_int>(dim);
        const lapack_int l_nrhs = static_cast<lapack_int>(nrhs);
        lapack_int info = 0;

        /* DPOTRF( UPLO, N, A, LDA, INFO ) */
        /* cholesky factorization */
        const lapack_int dpotrf_ret = dpotrf_(uplo_opt, &l_s, mat_temp.data(), &l_s, &info, uplo_opt_size);
        (void)dpotrf_ret;

        if (info == 0)
        {
            // no error, copy b to x
            std::copy_n(b_vec.data(), (nrhs * dim), x_vec.data());
            dpotrs_(uplo_opt, &l_s, &l_nrhs, mat_temp.data(), &l_s, x_vec.data(), &l_s, &info, uplo_opt_size);
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

bool linalg_is_posdef(
    Span<const Real> mat, size_t dim,
    Span<Real> work)
{
    bool retval = false;
    if ((dim > 0U) && (dim < (static_cast<size_t>(INT32_MAX) / dim)))
    {
        const size_t a_len = dim * dim;
        if (work.size() >= a_len)
        {
            // attempt cholesky factorization
            const LinalgErrorType err = linalg_cholesky_decomposition(
                mat.active(a_len), dim, work.active(a_len));
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
    Span<const Real> mat, size_t dim,
    Span<const Real> y_vec, size_t nrhs,
    Span<Real> work,
    Span<lapack_int> iwork,
    Span<Real> x_vec)
{
    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;
    /* alias data */
    if ((dim == 0U) || (dim >= (static_cast<size_t>(INT32_MAX) / dim)))
    {
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else if (work.size() < (dim * dim) || (iwork.size() < dim))
    {
        // not enough buffer for copy of A or y
        retval = LinalgErrorType::ERROR_MEMORY;
    }
    else
    {
        const size_t a_len = dim * dim;
        Span<Real> mat_lu = work.active(a_len);
        const lapack_int l_s = static_cast<lapack_int>(dim);
        // copy mat to buffer
        std::copy_n(mat.data(), a_len, mat_lu.data());

        /* DGETRF( M, N, A, LDA, IPIV, INFO ) */
        lapack_int info = 0;
        const lapack_int dgetrf_ret = dgetrf_(&l_s, &l_s, mat_lu.data(), &l_s, iwork.data(), &info);
        (void)dgetrf_ret;

        if (info == 0)
        {
            const char*      transpose_opt = "N";
            const size_t     transpose_opt_size = 1U;
            const lapack_int l_nrhs = static_cast<lapack_int>(nrhs);
            // copy y to x vector
            std::copy_n(y_vec.data(), (nrhs * dim), x_vec.data());
            //      SUBROUTINE DGETRS( TRANS, N, NRHS, A, LDA, IPIV, B, LDB, INFO )
            const lapack_int dgetrs_ret = dgetrs_(transpose_opt, &l_s, &l_nrhs, mat_lu.data(), &l_s, iwork.data(), x_vec.data(), &l_s, &info, transpose_opt_size);
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
    Span<const Real> mat, size_t rows, size_t cols,
    Span<Real> work,
    Span<Real> inv_mat)
{
    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;

    if ((rows == 0U) || (cols == 0U))
    {
        return LinalgErrorType::ERROR_INPUT;
    }
    if (rows >= (static_cast<size_t>(INT32_MAX) / cols))
    {
        return LinalgErrorType::ERROR_INPUT;
    }

    // Calculate the minimum dimension
    const size_t min_dim = (rows < cols) ? rows : cols;

    // Allocate working memory:
    // - For U matrix: rows × min_dim
    // - For S vector: min_dim
    // - For V^T matrix: min_dim × cols
    const size_t size_u = rows * min_dim;
    const size_t size_s = min_dim;
    const size_t size_vt = min_dim * cols;

    // Determine if we have enough work space
    const size_t size_usvt_work = size_u + size_s + size_vt;
    if (work.size() < size_usvt_work)
    {
        return LinalgErrorType::ERROR_MEMORY;
    }

    // Partition the work buffer
    Span<Real> u_mat = work.active(size_u);
    Span<Real> s_vec = work.active(size_s, size_u);
    Span<Real> vt_mat = work.active(size_vt, size_u + size_s);

    // Use remaining workspace for SVD computation
    Span<Real> svd_work = work.active(work.size() - size_usvt_work, size_usvt_work);

    // Perform SVD
    retval = linalg_svd(
        mat, rows, cols,
        false, false,
        svd_work,
        u_mat,
        s_vec,
        vt_mat);
    if (retval != LinalgErrorType::ERROR_NONE)
    {
        return retval;
    }

    const Real epsilon = 1.0e-12;  // Small threshold for numerical stability

    // Initialize the output inverse matrix with zeros
    std::fill_n(inv_mat.data(), cols * rows, 0.0);

    // Accumulate: inv_mat(i,j) += V(i,k) * (1/s_k) * U(j,k)
    // where inv_mat is (cols x rows).
    for (size_t k = 0U; k < min_dim; ++k)
    {
        const Real sval = s_vec[k];
        if (sval <= epsilon)
        {
            continue;
        }
        const Real inv_s = 1.0 / sval;

        // V(i,k) can be read from V^T(k,i) stored in vt_mat (col-major): vt_mat[i*min_dim + k]
        for (size_t i = 0U; i < cols; ++i)
        {
            const Real v_ik = vt_mat[i * min_dim + k];
            const Real scale = v_ik * inv_s;

            // U(j,k) is u_mat[k*rows + j] (col-major)
            for (size_t j = 0U; j < rows; ++j)
            {
                inv_mat[j * cols + i] += scale * u_mat[k * rows + j];
            }
        }
    }

    return retval;
}

LinalgErrorType linalg_leastsquares_solve(
    Span<const Real> mat, size_t rows, size_t cols,
    Span<const Real> b_vec, size_t nrhs,
    Span<Real> work,
    Span<Real> x_vec)
{
    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;

    if (rows == 0U || cols == 0U || nrhs == 0U ||
        (rows >= (static_cast<size_t>(INT32_MAX) / cols) || (nrhs >= static_cast<size_t>(INT32_MAX) / cols) || (nrhs >= static_cast<size_t>(INT32_MAX) / rows)))
    {
        // Input dimensions are too large
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else if (work.size() < ((rows * cols) + (FSB_MAX(rows, cols) * nrhs)))
    {
        retval = LinalgErrorType::ERROR_MEMORY;
    }
    else
    {
        // Calculate required work space:
        // - For A matrix: rows * cols
        const size_t size_a = rows * cols;
        const size_t size_ldb = FSB_MAX(rows, cols);
        const size_t size_b_work = size_ldb * nrhs;
        const size_t work_len_ab = size_a + size_b_work;
        // Partition the work buffer for mat A
        Span<Real> a_copy = work.active(size_a);
        Span<Real> b_work = work.active(size_b_work, size_a);
        // Copy matrix A to work buffer
        std::copy_n(mat.data(), size_a, a_copy.data());
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
        const lapack_int l_cols = static_cast<lapack_int>(cols);
        const lapack_int l_nrhs = static_cast<lapack_int>(nrhs);
        const lapack_int lda = l_rows;
        const lapack_int ldb = static_cast<lapack_int>(size_ldb);  // leading dimension of b
        lapack_int info = 0;

        // Query workspace size
        Real work_query = 0.0;
        lapack_int lwork = -1;
        dgels_(trans_opt, &l_rows, &l_cols, &l_nrhs,
               a_copy.data(), &lda, b_work.data(), &ldb,
               &work_query, &lwork, &info,
               trans_opt_size);
        if (info != 0)
        {
            retval = LinalgErrorType::ERROR_QUERY;
        }
        else
        {
            lwork = static_cast<lapack_int>(work_query);
            const auto s_lwork = static_cast<size_t>(lwork);
            // Check if we have enough workspace
            if (work.size() < (work_len_ab + s_lwork))
            {
                retval = LinalgErrorType::ERROR_MEMORY;
            }
        }

        // Solve with dgels
        if (retval == LinalgErrorType::ERROR_NONE)
        {
            lwork = static_cast<lapack_int>(work_query);
            const auto s_lwork = static_cast<size_t>(lwork);
            // Allocate workspace for dgels
            Span<Real> dgels_work = work.active(s_lwork, work_len_ab);
            // Solve least squares problem using dgels
            dgels_(trans_opt, &l_rows, &l_cols, &l_nrhs,
                   a_copy.data(), &lda, b_work.data(), &ldb,
                   dgels_work.data(), &lwork, &info,
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
            const size_t output_rows = cols;
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

LinalgErrorType linalg_sym_lt_eig(
    Span<const Real> mat, size_t dim,
    Span<Real> work,
    Span<Real> val,
    Span<Real> vec)
{
    LinalgErrorType retval = LinalgErrorType::ERROR_NONE;
    if ((dim == 0U) || (dim >= (static_cast<size_t>(INT32_MAX) / dim)))
    {
        retval = LinalgErrorType::ERROR_INPUT;
    }
    else
    {
        const size_t a_len = dim * dim;

        // copy A to output vector
        std::copy_n(mat.data(), a_len, vec.data());

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
               &l_s, vec.data(), &l_s,
               val.data(),
               &work_query, &lwork, &info,
               job_opt_size, uplo_opt_size);
        lwork = static_cast<lapack_int>(work_query);
        const auto s_lwork = static_cast<size_t>(lwork);

        if (info != 0)
        {
            /* dsyev_ work query failed */
            retval = LinalgErrorType::ERROR_QUERY;
        }
        else if (work.size() < s_lwork)
        {
            // not enough buffer for operation
            retval = LinalgErrorType::ERROR_MEMORY;
        }
        else
        {
            Span<Real> sym_work = work.active(s_lwork);

            // perform operation
            dsyev_(job_opt, uplo_opt,
                &l_s, vec.data(), &l_s,
                val.data(),
                sym_work.data(), &lwork, &info,
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

} // namespace fsb
