#include <algorithm>
#include <cmath>
#include <cmath>
#include "fsb_types.h"
#include "fsb_linalg3.h"

#include "fsb_motion.h"

namespace fsb {

static void ortho_compliment(const Vec3& vec_w, Vec3& vec_u, Vec3& vec_v)
{
    // Robustly compute a right-handed orthonormal set { vec_u, V, W }.
    // The vector W is guaranteed to be unit-length, in which case
    // there is no need to worry about a division by zero when
    // computing invLength.
    if (fabs(vec_w.x) > fabs(vec_w.y))
    {
        // The component of maximum absolute value is either vec_w.x
        // or vec_w.z.
        const Real den = sqrt(vec_w.x * vec_w.x + vec_w.z * vec_w.z);
        vec_u = {-vec_w.z / den, 0.0, vec_w.x / den};
    }
    else
    {
        // The component of maximum absolute value is either vec_w.y
        // or vec_w.z.
        const Real den = sqrt(vec_w.y * vec_w.y + vec_w.z * vec_w.z);
        vec_u = {0.0, vec_w.z / den, -vec_w.y / den};
    }
    vec_v = vector_cross(vec_w, vec_u);
}

static Vec3 eigenvector1(const Mat3Sym& mat, const Vec3& evec0, const Real eva_l1)
{
    Vec3 result;
    Vec3 vec_u = {};
    Vec3 vec_v = {};
    ortho_compliment(evec0, vec_u, vec_v);

    // lower-triangular column-major
    const Real& mat_xx = mat.m00;
    const Real& mat_xy = mat.m01;
    const Real& mat_xz = mat.m02;
    const Real& mat_yy = mat.m11;
    const Real& mat_yz = mat.m12;
    const Real& mat_zz = mat.m22;

    // Let e be eval1 and let E be a corresponding eigenvector which
    // is a solution to the linear system (A - e*I)*E = 0.  The matrix
    // (A - e*I) is 3x3, not invertible (so infinitely many
    // solutions), and has rank 2 when eval1 and eval are different.
    // It has rank 1 when eval1 and eval2 are equal.  Numerically, it
    // is difficult to compute robustly the rank of a matrix.  Instead,
    // the 3x3 linear system is reduced to a 2x2 system as follows.
    // Define the 3x2 matrix J = [U V] whose columns are the U and V
    // computed previously.  Define the 2x1 vector X = J*E.  The 2x2
    // system is 0 = M * X = (J^T * (A - e*I) * J) * X where J^T is
    // the transpose of J and M = J^T * (A - e*I) * J is a 2x2 matrix.
    // The system may be written as
    //     +-                        -++-  -+       +-  -+
    //     | U^T*A*U - e  U^T*A*V     || x0 | = e * | x0 |
    //     | V^T*A*U      V^T*A*V - e || x1 |       | x1 |
    //     +-                        -++   -+       +-  -+
    // where X has row entries x0 and x1.
    const Vec3 a_u
        = {mat_xx * vec_u.x + mat_xy * vec_u.y + mat_xz * vec_u.z,
           mat_xy * vec_u.x + mat_yy * vec_u.y + mat_yz * vec_u.z,
           mat_xz * vec_u.x + mat_yz * vec_u.y + mat_zz * vec_u.z};
    const Vec3 a_v
        = {mat_xx * vec_v.x + mat_xy * vec_v.y + mat_xz * vec_v.z,
           mat_xy * vec_v.x + mat_yy * vec_v.y + mat_yz * vec_v.z,
           mat_xz * vec_v.x + mat_yz * vec_v.y + mat_zz * vec_v.z};
    Real m00 = vec_u.x * a_u.x + vec_u.y * a_u.y + (vec_u.z * a_u.z - eva_l1);
    Real m01 = vec_u.x * a_v.x + vec_u.y * a_v.y + vec_u.z * a_v.z;
    Real m11 = vec_v.x * a_v.x + vec_v.y * a_v.y + (vec_v.z * a_v.z - eva_l1);

    // For robustness, choose the largest-length row of M to compute
    // the eigenvector.  The 2-tuple of coefficients of U and V in the
    // assignments to eigenvector[1] lies on a circle, and U and V are
    // unit length and perpendicular, so eigenvector[1] is unit length
    // (within numerical tolerance).
    const Real abs_m00 = fabs(m00);
    const Real abs_m01 = fabs(m01);
    const Real abs_m11 = fabs(m11);
    if ((abs_m00 < FSB_TOL) && (abs_m01 < FSB_TOL) && (abs_m11 < FSB_TOL))
    {
        result = vec_u;
    }
    else if (abs_m00 >= abs_m11)
    {
        if (abs_m00 >= abs_m01)
        {
            m01 /= m00;
            m00 = 1.0 / sqrt(1.0 + m01 * m01);
            m01 *= m00;
        }
        else
        {
            m00 /= m01;
            m01 = 1.0 / sqrt(1.0 + m00 * m00);
            m00 *= m01;
        }
        result = vector_subtract(vector_scale(m01, vec_u), vector_scale(m00, vec_v));
    }
    else
    {
        if (abs_m11 >= abs_m01)
        {
            m01 /= m11;
            m11 = 1.0 / sqrt(1.0 + m01 * m01);
            m01 *= m11;
        }
        else
        {
            m11 /= m01;
            m01 = 1.0 / sqrt(1.0 + m11 * m11);
            m11 *= m01;
        }
        result = vector_subtract(vector_scale(m11, vec_u), vector_scale(m01, vec_v));
    }

    return result;
}

static Vec3 eigenvector0(const Mat3Sym& mat, const Real eva_l0)
{
    Vec3 result;

    // lower-triangular column-major
    const Real& mat_xx = mat.m00;
    const Real& mat_xy = mat.m01;
    const Real& mat_xz = mat.m02;
    const Real& mat_yy = mat.m11;
    const Real& mat_yz = mat.m12;
    const Real& mat_zz = mat.m22;

    const Vec3   ro_w0 = {mat_xx - eva_l0, mat_xy, mat_xz};
    const Vec3   ro_w1 = {mat_xy, mat_yy - eva_l0, mat_yz};
    const Vec3   ro_w2 = {mat_xz, mat_yz, mat_zz - eva_l0};
    const Vec3   ro_w0_ro_w1 = vector_cross(ro_w0, ro_w1);
    const Vec3   ro_w0_ro_w2 = vector_cross(ro_w0, ro_w2);
    const Vec3   ro_w1_ro_w2 = vector_cross(ro_w1, ro_w2);
    const Real ma_g0 = vector_dot(ro_w0_ro_w1, ro_w0_ro_w1);
    const Real ma_g1 = vector_dot(ro_w0_ro_w2, ro_w0_ro_w2);
    const Real ma_g2 = vector_dot(ro_w1_ro_w2, ro_w1_ro_w2);

    // select the two rows with cross product that has the largest length
    // of all pairs of rows.
    if ((ma_g0 > ma_g1) && (ma_g0 > ma_g2))
    {
        const Real mag = sqrt(ma_g0);
        result = {ro_w0_ro_w1.x / mag, ro_w0_ro_w1.y / mag, ro_w0_ro_w1.z / mag};
    }
    else if ((ma_g1 > ma_g0) && (ma_g1 > ma_g2))
    {
        const Real mag = sqrt(ma_g1);
        result = {ro_w0_ro_w2.x / mag, ro_w0_ro_w2.y / mag, ro_w0_ro_w2.z / mag};
    }
    else
    {
        const Real mag = sqrt(ma_g2);
        result = {ro_w1_ro_w2.x / mag, ro_w1_ro_w2.y / mag, ro_w1_ro_w2.z / mag};
    }

    return result;
}

static Real precondition_matrix_scale(Mat3Sym& mat)
{
    Real scale = fabs(mat.m00);
    scale = std::max(fabs(mat.m01), scale);
    scale = std::max(fabs(mat.m02), scale);
    scale = std::max(fabs(mat.m11), scale);
    scale = std::max(fabs(mat.m12), scale);
    scale = std::max(fabs(mat.m22), scale);

    if (scale < FSB_TOL)
    {
        scale = 0.0;
        mat.m00 = 0.0;
        mat.m01 = 0.0;
        mat.m02 = 0.0;
        mat.m11 = 0.0;
        mat.m12 = 0.0;
        mat.m22 = 0.0;
    }
    else
    {
        mat.m00 = mat.m00 / scale;
        mat.m01 = mat.m01 / scale;
        mat.m02 = mat.m02 / scale;
        mat.m11 = mat.m11 / scale;
        mat.m12 = mat.m12 / scale;
        mat.m22 = mat.m22 / scale;
    }

    return scale;
}

bool mat3_posdef_symmetric_eigenvalues(const Mat3Sym& mat, Vec3& eigenvalues)
{
    // initialize zero
    eigenvalues = {};
    if (const Real norm = (mat.m01 * mat.m01) + (mat.m02 * mat.m02) + (mat.m12 * mat.m12);
        norm < FSB_TOL)
    {
        eigenvalues.x = mat.m00;
        eigenvalues.y = mat.m11;
        eigenvalues.z = mat.m22;
    }
    else
    {
        // In the PDF mentioned previously, B = (A - q*I)/p, where
        // q = tr(A)/3 with tr(A) the trace of A (sum of the diagonal
        // entries of A) and where p = sqrt(tr((A - q*I)^2)/6).
        const Real tr_3 = (mat.m00 + mat.m11 + mat.m22) / 3.0;
        // The matrix A - q*I is represented by the following, where
        // b00, b11 and b22 are computed after these comments,
        //   +-           -+
        //   | b00 mat.m01 mat.m02 |
        //   | mat.m01 b11 mat.m12 |
        //   | mat.m02 mat.m12 b22 |
        //   +-           -+
        const Real b00 = mat.m00 - tr_3;
        const Real b11 = mat.m11 - tr_3;
        const Real b22 = mat.m22 - tr_3;
        // This is the variable p mentioned in the PDF.
        const Real p00 = b00 * b00 + b11 * b11 + b22 * b22 + norm * 2.0;
        const Real p01 = sqrt(p00 / 6.0);
        // We need det(B) = det((A - q*I)/p) = det(A - q*I)/p^3.  The
        // value det(A - q*I) is computed using a cofactor expansion
        // by the first row of A - q*I.  The cofactors are c00, c01
        // and c02 and the determinant is b00*c00 - mat.m01*c01 + mat.m02*c02.
        // The det(B) is then computed finally by the division
        // with p^3.
        const Real c00 = b11 * b22 - mat.m12 * mat.m12;
        const Real c01 = mat.m01 * b22 - mat.m12 * mat.m02;
        const Real c02 = mat.m01 * mat.m12 - b11 * mat.m02;

        const Real det = (b00 * c00 - mat.m01 * c01 + mat.m02 * c02) / (p01 * p01 * p01);
        // The det_2 value is cos(3*theta) mentioned in the PDF. The
        // acos(z) function requires |z| <= 1, but will fail silently
        // and return NaN if the input is larger than 1 in magnitude.
        // To avoid this problem due to rounding errors, the det_2
        // value is clamped to [-1,1].
        Real det_2 = det / 2.0;
        if (det_2 < -1.0)
        {
            det_2 = -1.0;
        }
        else if (det_2 > 1.0)
        {
            det_2 = 1.0;
        }
        else
        {
            // keep det / 2.0
        }

        // The eigenvalues of B are ordered as
        // beta0 <= beta1 <= beta2.  The number of digits in
        // twoThirdsPi is chosen so that, whether float or double,
        // the floating-point number is the closest to theoretical
        // 2*pi/3.
        const Real angle = acos(det_2) / 3.0;

        const Real bet_a2 = cos(angle) * 2.0;
        const Real bet_a0 = cos(angle + 2.0 * M_PI / 3.0) * 2.0;
        const Real bet_a1 = -(bet_a0 + bet_a2);
        // The eigenvalues of A are ordered as
        // alpha0 <= alpha1 <= alpha2.
        eigenvalues.x = tr_3 + p01 * bet_a0;
        eigenvalues.y = tr_3 + p01 * bet_a1;
        eigenvalues.z = tr_3 + p01 * bet_a2;
    }

    const bool is_pd
        = ((eigenvalues.x > FSB_TOL) && (eigenvalues.y > FSB_TOL) && (eigenvalues.z > FSB_TOL));
    return is_pd;
}

bool mat3_posdef_symmetric_eigenvectors(const Mat3Sym& mat_in, Vec3& eigenvalues, Vec3& eig_vec0, Vec3& eig_vec1, Vec3& eig_vec2)
{
    Mat3Sym mat = mat_in;
    const Real mat_scale = precondition_matrix_scale(mat);

    const bool is_pd = mat3_posdef_symmetric_eigenvalues(mat, eigenvalues);
    if (is_pd)
    {
        eig_vec0 = eigenvector0(mat, eigenvalues.x);
        eig_vec1 = eigenvector1(mat, eig_vec0, eigenvalues.y);
        eig_vec2  = vector_cross(eig_vec0, eig_vec1);
    }

    eigenvalues.x = mat_scale * eigenvalues.x;
    eigenvalues.y = mat_scale * eigenvalues.y;
    eigenvalues.z = mat_scale * eigenvalues.z;

    return is_pd;
}

}
