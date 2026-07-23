// Replaces lgsm (thirdparty/lgsm) per the user's decision recorded in
// PORTING_NOTES.md Phase 2: lgsm's custom Eigen-expression classes predate
// Eigen's "evaluator" system (Eigen 3.3+) and are fundamentally
// incompatible with every Eigen version available today (their inherited
// generic MatrixBase machinery — operator(), and by extension most
// arithmetic — does not compile). This header re-derives the small set of
// SE(3)/so(3) operations OMIP actually uses (twist storage, log, exp,
// adjoint) as plain functions over Eigen::Isometry3d/Eigen::Vector3d, with
// no custom expression-template class, verified against lgsm's own source
// formulas line-by-line (see LieGroup_SO3.h, LieGroup_SE3.h,
// LieAlgebra_so3.h in the git history of thirdparty/lgsm) rather than
// re-derived from scratch, specifically to preserve bit-for-bit numerical
// behavior including two non-obvious details:
//  1. lgsm was built with USE_RLAB_LOG_FUNCTION defined, so Displacementd
//     ::log() used the "RLab" dexpinv formula (numerically stable near
//     theta=pi), not the naive sin/cos formula that also exists
//     side-by-side in lgsm's source guarded out by that same macro.
//  2. lgsm's se(3) LieAlgebraBase::exp(precision) never actually forwards
//     its `precision` parameter to the so(3) exp()/dexp() calls it makes
//     internally, which use their own hardcoded defaults (1e-5 for exp,
//     1e-6/1e-2 for dexp) regardless of what the caller passed. This is
//     preserved deliberately below (se3Exp ignores its precision
//     parameter) rather than "fixed", since fixing it would be a silent
//     numerical behavior change relative to the original ROS build.
#ifndef OMIP_CORE_LIE_GROUP_HPP_
#define OMIP_CORE_LIE_GROUP_HPP_

#include <cmath>

#include <Eigen/Geometry>

namespace omip
{

// Replaces Eigen::Twistd (lgsm). An se(3) element: angular velocity
// (rx,ry,rz) then linear velocity (vx,vy,vz), matching lgsm's own
// [angular; linear] storage order and accessor names exactly.
class Twistd
{
public:
    Twistd() : m_coeffs(Eigen::Matrix<double, 6, 1>::Zero()) {}
    Twistd(double rx, double ry, double rz, double vx, double vy, double vz)
    {
        m_coeffs << rx, ry, rz, vx, vy, vz;
    }
    explicit Twistd(const Eigen::Matrix<double, 6, 1>& coeffs) : m_coeffs(coeffs) {}
    Twistd(const Eigen::Vector3d& angular, const Eigen::Vector3d& linear)
    {
        m_coeffs.head<3>() = angular;
        m_coeffs.tail<3>() = linear;
    }

    double rx() const { return m_coeffs(0); }
    double ry() const { return m_coeffs(1); }
    double rz() const { return m_coeffs(2); }
    double vx() const { return m_coeffs(3); }
    double vy() const { return m_coeffs(4); }
    double vz() const { return m_coeffs(5); }
    double& rx() { return m_coeffs(0); }
    double& ry() { return m_coeffs(1); }
    double& rz() { return m_coeffs(2); }
    double& vx() { return m_coeffs(3); }
    double& vy() { return m_coeffs(4); }
    double& vz() { return m_coeffs(5); }

    Eigen::Vector3d getAngularVelocity() const { return m_coeffs.head<3>(); }
    Eigen::Vector3d getLinearVelocity() const { return m_coeffs.tail<3>(); }

    // Replaces the .norm() inherited from Eigen::MatrixBase in lgsm's
    // Eigen::Twistd (which derived from a generic 6x1 Eigen expression
    // type) — plain Euclidean norm of the 6 stacked coefficients.
    double norm() const { return m_coeffs.norm(); }

    const Eigen::Matrix<double, 6, 1>& coeffs() const { return m_coeffs; }
    Eigen::Matrix<double, 6, 1>& coeffs() { return m_coeffs; }

    Twistd operator-() const { return Twistd((-m_coeffs).eval()); }
    Twistd operator+(const Twistd& o) const { return Twistd((m_coeffs + o.m_coeffs).eval()); }
    Twistd operator-(const Twistd& o) const { return Twistd((m_coeffs - o.m_coeffs).eval()); }
    Twistd operator*(double s) const { return Twistd((m_coeffs * s).eval()); }
    Twistd operator/(double s) const { return Twistd((m_coeffs / s).eval()); }
    Twistd& operator/=(double s) { m_coeffs /= s; return *this; }

private:
    Eigen::Matrix<double, 6, 1> m_coeffs;
};

inline Twistd operator*(double s, const Twistd& t) { return t * s; }

// Replaces the implicit Eigen::Matrix<double,6,6> * Eigen::Twistd(lgsm)
// multiplication used in joint_tracker (e.g. `adjoint * twist` in
// PrismaticJointFilter/RevoluteJointFilter's getPredictedSRB*() methods).
// lgsm's Twistd had a `Twist(const MatrixBase<OtherDerived>&)` constructor
// that copied a generic 6-vector expression's coefficients directly into
// m_coeffs (verified in lgsm's own Twist.h, git history) — i.e. purely
// mechanical, position-for-position, with no awareness of the
// angular-first/linear-first semantic split. This free function replicates
// exactly that: raw 6x6-matrix-times-6-vector on coeffs(), independent of
// whether the matrix (e.g. from computeAdjoint(), OMIPUtils.cpp) was
// documented as expecting a different ordering — same computation the
// original code performed, preserved as-is (see PORTING_NOTES.md Phase 4).
inline Twistd operator*(const Eigen::Matrix<double, 6, 6>& m, const Twistd& t)
{
    return Twistd((m * t.coeffs()).eval());
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d& w)
{
    Eigen::Matrix3d m;
    m << 0, -w.z(), w.y(),
         w.z(), 0, -w.x(),
        -w.y(), w.x(), 0;
    return m;
}

// so(3) exponential map: axis-angle vector -> unit quaternion.
// Verbatim port of LieAlgebraBase<Matrix<Scalar,3,1>,Derived>::exp()
// (lgsm/LieAlgebra_so3.h), hardcoded precision = 1e-5 to match lgsm's
// default (the only value ever used, see file header note).
inline Eigen::Quaterniond so3Exp(const Eigen::Vector3d& w, double precision = 1e-5)
{
    const double n2 = w.squaredNorm();
    double qw, sinc_half;
    if (n2 < precision)
    {
        qw = 1.0 + (-1.0 + n2 / 48.0) * n2 / 8.0;
        sinc_half = (1.0 + (-1.0 + 0.0125 * n2) * n2 / 24.0) / 2.0;
    }
    else
    {
        const double n = std::sqrt(n2);
        qw = std::cos(n * 0.5);
        sinc_half = std::sin(n * 0.5) / n;
    }
    const Eigen::Vector3d qv = sinc_half * w;
    return Eigen::Quaterniond(qw, qv.x(), qv.y(), qv.z());
}

// First derivative of the so(3) exponential map ("V" matrix). Verbatim
// port of LieAlgebraBase<Matrix<Scalar,3,1>,Derived>::dexp()
// (lgsm/LieAlgebra_so3.h), hardcoded precision/precision2 = 1e-6/1e-2 to
// match lgsm's default (the only values ever used, see file header note).
inline Eigen::Matrix3d so3Dexp(const Eigen::Vector3d& w, double precision = 1e-6, double precision2 = 1e-2)
{
    const double n2 = w.squaredNorm();
    const double n = std::sqrt(n2);
    const double sin_n = std::sin(n);
    const double cos_n = std::cos(n);

    const double f2 = (n2 < precision) ? (0.5 + (-1.0 + n2 / 30.0) * n2 / 24.0) : (1.0 - cos_n) / n2;
    const double f3 = (n2 < precision2) ? (20.0 + (-1.0 + n2 / 42.0) * n2) / 120.0 : (n - sin_n) / (n2 * n);

    const Eigen::Matrix3d W = skew(w);
    return Eigen::Matrix3d::Identity() + f2 * W + f3 * (W * W);
}

// se(3) exponential map: twist -> rigid transform. Verbatim port of
// LieAlgebraBase<Matrix<Scalar,6,1>,Derived>::exp() (lgsm/LieAlgebra_se3.h).
// The `precision` parameter is intentionally unused, replicating lgsm's own
// behavior (see file header note 2) — not a bug introduced here.
inline Eigen::Isometry3d se3Exp(const Twistd& xi, double /*precision*/ = 1e-12)
{
    const Eigen::Vector3d w = xi.getAngularVelocity();
    const Eigen::Vector3d v = xi.getLinearVelocity();
    Eigen::Isometry3d g = Eigen::Isometry3d::Identity();
    g.linear() = so3Exp(w).toRotationMatrix();
    g.translation() = so3Dexp(w).transpose() * v;
    return g;
}

// se(3) logarithm map: rigid transform -> twist. Verbatim port of the
// "RLab" branch of LieGroupBase<Array<Scalar,7,1>,Derived>::log()
// (lgsm/LieGroup_SE3.h) — this is the branch that was actually active in
// the original build (USE_RLAB_LOG_FUNCTION was defined), chosen by the
// original authors for numerical stability near a rotation angle of pi.
inline Twistd se3Log(const Eigen::Isometry3d& g, double precision = 1e-12)
{
    const Eigen::Quaterniond q(Eigen::Matrix3d(g.linear()));

    // so(3) logarithm (LieGroupBase<Quaternion<Scalar>,Derived>::log()).
    Eigen::Vector3d ang;
    {
        const double n2 = q.vec().squaredNorm();
        const double n = std::sqrt(n2);
        if (n < precision)
            ang = (2.0 / q.w()) * q.vec();
        else
            ang = (std::atan2(2 * n * q.w(), q.w() * q.w() - n2) / n) * q.vec();
    }

    const double n2 = ang.squaredNorm();
    Eigen::Vector3d lin;
    if (n2 < precision)
    {
        lin = g.translation();
    }
    else
    {
        const double n = std::sqrt(n2);
        const double n_div2 = n / 2.0;
        const double s = std::sin(n_div2) / n_div2;
        const double c = std::cos(n_div2);
        const double gamma = c / s;

        const Eigen::Matrix3d w_ceil = skew(ang);
        const Eigen::Matrix3d dexpinv = Eigen::Matrix3d::Identity() - 0.5 * w_ceil
            + (1.0 - gamma) / n2 * (w_ceil * w_ceil);

        lin = dexpinv * g.translation();
    }

    return Twistd(ang, lin);
}

// SE(3) adjoint representation. Verbatim port of
// LieGroupBase<Array<Scalar,7,1>,Derived>::adjoint() (lgsm/LieGroup_SE3.h),
// same [angular;linear]-block layout lgsm used
// ( Ad_H = [[R,0],[skew(t)*R, R]] ) — omip::computeAdjoint() (OMIPUtils.cpp)
// swaps this into OMIP's own [linear;angular] convention, unchanged.
inline Eigen::Matrix<double, 6, 6> se3Adjoint(const Eigen::Isometry3d& g)
{
    Eigen::Matrix<double, 6, 6> res = Eigen::Matrix<double, 6, 6>::Zero();
    const Eigen::Matrix3d R = g.linear();
    res.block<3, 3>(0, 0) = R;
    res.block<3, 3>(3, 3) = R;
    res.block<3, 3>(3, 0) = skew(g.translation()) * R;
    return res;
}

} // namespace omip

#endif // OMIP_CORE_LIE_GROUP_HPP_
