/**
 * ============================================================
 *
 *  COORDINATE FRAMES
 *  ─────────────────
 *  Navigation frame (NED):  X=North  Y=East  Z=Down
 *  Body frame:              X=Bow    Y=Starboard  Z=Keel-down
 *
 *  ATTITUDE CONVENTION
 *  ───────────────────
 *  Roll  (φ): rotation about X-axis
 *  Pitch (θ): rotation about Y-axis
 *  Yaw   (ψ): rotation about Z-axis
 *  Applied in ZYX order (yaw → pitch → roll)
 *
 *  NONLINEAR MOTION MODEL
 *  ──────────────────────
 *  ṗ = R_bn · v_body
 *
 *  v̇ = R_bn · a_body − [0, 0, g]ᵀ
 *    (gravity subtracted in navigation frame)
 *
 *  η̇ = W(φ,θ) · ω_body
 *    where W is the Euler angle kinematics matrix
 *
 *  JACOBIAN (EKF LINEARISATION)
 *  ──────────────────────────────
 *  F = ∂f/∂x, computed analytically for all nonzero partial
 *  derivatives involving roll/pitch/yaw in the kinematics.
 * ============================================================
 */

#include "ekf.hpp"
#include <cmath>
#include <stdexcept>

namespace auv {

// ── Gravity constant ──────────────────────────────────────────
static constexpr double G_MSS = 9.81;   // m/s²

// ── Helper macro for vector element access ────────────────────
#define V(vec, i) (vec)(i, 0)

// ─────────────────────────────────────────────────────────────
//  Static helpers
// ─────────────────────────────────────────────────────────────

double EKF::wrapAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

/**
 *  Rotation matrix body → navigation (NED)
 *  Using ZYX (yaw-pitch-roll) Euler angles:
 *
 *  R_bn = Rz(ψ) · Ry(θ) · Rx(φ)
 *
 *  [ cψcθ   cψsθsφ−sψcφ   cψsθcφ+sψsφ ]
 *  [ sψcθ   sψsθsφ+cψcφ   sψsθcφ−cψsφ ]
 *  [ -sθ    cθsφ           cθcφ         ]
 */
Mat3 EKF::bodyToNav(double roll, double pitch, double yaw) {
    double cr = std::cos(roll),  sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw),   sy = std::sin(yaw);

    Mat3 R;
    R(0,0) =  cy*cp;
    R(0,1) =  cy*sp*sr - sy*cr;
    R(0,2) =  cy*sp*cr + sy*sr;
    R(1,0) =  sy*cp;
    R(1,1) =  sy*sp*sr + cy*cr;
    R(1,2) =  sy*sp*cr - cy*sr;
    R(2,0) = -sp;
    R(2,1) =  cp*sr;
    R(2,2) =  cp*cr;
    return R;
}

/**
 *  Euler angle kinematics matrix W(φ,θ):
 *
 *  [φ̇]   [ 1  sinφ·tanθ  cosφ·tanθ ] [gx]
 *  [θ̇] = [ 0  cosφ       -sinφ      ] [gy]
 *  [ψ̇]   [ 0  sinφ/cosθ  cosφ/cosθ  ] [gz]
 *
 *  Singularity at θ = ±90° (gimbal lock).
 *  For underwater AUVs this is acceptable (pitch rarely ±90°).
 */
Mat3 EKF::eulerKinMatrix(double roll, double pitch) {
    double sr = std::sin(roll),  cr = std::cos(roll);
    double tp = std::tan(pitch), cp = std::cos(pitch);

    // Guard against gimbal lock
    if (std::abs(cp) < 1e-6) cp = 1e-6;

    Mat3 W;
    W(0,0) = 1.0;  W(0,1) = sr*tp;    W(0,2) = cr*tp;
    W(1,0) = 0.0;  W(1,1) = cr;       W(1,2) = -sr;
    W(2,0) = 0.0;  W(2,1) = sr/cp;    W(2,2) = cr/cp;
    return W;
}

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────

EKF::EKF(double sigma_acc, double sigma_gyro,
        double sigma_dvl, double sigma_depth,
        double sigma_gps, double sigma_mag,
        double sigma_proc)
    : prev_px_(0), prev_py_(0), prev_pz_(0),
    distance_total_(0),
    last_gx_(0), last_gy_(0), last_gz_(0),
    last_ax_(0), last_ay_(0), last_az_(0),
    initialised_(false)
{
    x_ = Vec9::Zero();
    P_ = Mat9::Identity();
    for (int i = 0; i < N; ++i) P_(i,i) = 1.0;

    // Process noise base (diagonal; buildQ scales by dt)
    Q_base_ = Mat9::Zero();
    double qp = sigma_proc * sigma_proc;
    for (int i = 0; i < N; ++i) Q_base_(i,i) = qp;

    // Measurement noise matrices
    auto fill3 = [](Mat3& M, double s) {
        double v = s*s; M = Mat3::Zero();
        for (int i = 0; i < 3; ++i) M(i,i) = v;
    };
    fill3(R_acc_, sigma_acc);
    fill3(R_dvl_, sigma_dvl);
    R_depth_(0,0) = sigma_depth * sigma_depth;
    R_gps_(0,0)   = sigma_gps   * sigma_gps;
    R_gps_(1,1)   = sigma_gps   * sigma_gps;
    R_mag_(0,0)   = sigma_mag   * sigma_mag;
}

// ─────────────────────────────────────────────────────────────
//  Initialise
// ─────────────────────────────────────────────────────────────

void EKF::initialise(const IMUData& imu, const DVLData& dvl, double initial_depth, double initial_yaw_rad)
{
    x_ = Vec9::Zero();

    // Estimate initial roll and pitch from gravity vector
    // (static accelerometer reading ≈ -g in body frame)
    double ax = imu.ax, ay = imu.ay, az = imu.az;
    double a_norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (a_norm > 0.1) {
        V(x_,7) = std::asin(-ax / a_norm);                       // pitch
        V(x_,6) = std::atan2(ay, az);                            // roll
    }
    V(x_,8) = initial_yaw_rad;                                    // yaw
    V(x_,2) = initial_depth;                                      // depth

    if (dvl.valid) {
        V(x_,3) = dvl.vx; V(x_,4) = dvl.vy; V(x_,5) = dvl.vz;
    }

    // Initial covariance: small for attitude (we bootstrapped it),
    // moderate for position (start is reasonably known)
    P_ = Mat9::Zero();
    for (int i = 0; i < 3; ++i) P_(i,i)   = 1.0;    // pos: ±1.0 m (more confident start)
    for (int i = 3; i < 6; ++i) P_(i,i)   = 0.04;   // vel: ±0.2 m/s (DVL initializes this)
    P_(6,6) = 0.005;  P_(7,7) = 0.005;               // roll/pitch: ±4° (good accel init)
    P_(8,8) = 0.05;                                   // yaw: ±13° (magnetometer provides good init)

    last_ax_ = ax; last_ay_ = ay; last_az_ = az;
    last_gx_ = imu.gx; last_gy_ = imu.gy; last_gz_ = imu.gz;
    prev_px_ = V(x_,0); prev_py_ = V(x_,1); prev_pz_ = V(x_,2);
    distance_total_ = 0;
    initialised_ = true;
}

// ─────────────────────────────────────────────────────────────
//  Nonlinear state transition  f(x, u, dt)
// ─────────────────────────────────────────────────────────────

Vec9 EKF::stateTransition(const Vec9& x,double ax, double ay, double az,double gx, double gy, double gz,double dt) const
{
    // Extract state
    double vx  = V(x,3), vy  = V(x,4), vz  = V(x,5);
    double phi = V(x,6), the = V(x,7), psi = V(x,8);

    // Body → Nav rotation
    Mat3 Rbn = bodyToNav(phi, the, psi);

    // ── Position update: p += R_bn · v_body · dt ─────────────
    Vec3 v_body; 
    V(v_body,0)=vx; 
    V(v_body,1)=vy; 
    V(v_body,2)=vz;

    Vec3 v_nav = Rbn * v_body;

    // ── Velocity update: v += (R_bn · a_body - g_nav) · dt ───
    Vec3 a_body; 
    V(a_body,0)=ax; 
    V(a_body,1)=ay; 
    V(a_body,2)=az;
    
    Vec3 a_nav = Rbn * a_body;
    // Subtract gravity (NED: gravity acts in +Z direction)
    V(a_nav,2) -= G_MSS;

    // ── Attitude update: η += W(φ,θ) · ω_body · dt ───────────
    Mat3 W = eulerKinMatrix(phi, the);
    Vec3 omega; V(omega,0)=gx; V(omega,1)=gy; V(omega,2)=gz;
    Vec3 deta = W * omega;

    Vec9 xn;
    V(xn,0) = V(x,0) + V(v_nav,0) * dt;
    V(xn,1) = V(x,1) + V(v_nav,1) * dt;
    V(xn,2) = V(x,2) + V(v_nav,2) * dt;
    V(xn,3) = vx + V(a_nav,0) * dt;
    V(xn,4) = vy + V(a_nav,1) * dt;
    V(xn,5) = vz + V(a_nav,2) * dt;
    V(xn,6) = wrapAngle(phi + V(deta,0) * dt);
    V(xn,7) = wrapAngle(the + V(deta,1) * dt);
    V(xn,8) = wrapAngle(psi + V(deta,2) * dt);
    return xn;
}

// ─────────────────────────────────────────────────────────────
//  Jacobian  F = ∂f/∂x  (9×9)
//  ─────────────────────────────────────────────────────────────
//  The nonzero off-diagonal blocks arise from:
//    ∂(ṗ)/∂v  = R_bn · dt            (3×3 block, rows 0-2, cols 3-5)
//    ∂(ṗ)/∂η  = (∂R_bn/∂η · v) · dt (3×3 block, rows 0-2, cols 6-8)
//    ∂(v̇)/∂η  = (∂R_bn/∂η · a) · dt (3×3 block, rows 3-5, cols 6-8)
//    ∂(η̇)/∂η  = (∂W/∂η · ω) · dt    (3×3 block, rows 6-8, cols 6-8)
// ─────────────────────────────────────────────────────────────

Mat9 EKF::computeJacobian(const Vec9& x,double ax, double ay, double az,double gx, double gy, double gz,double dt) const
{
    double vx  = V(x,3), vy  = V(x,4), vz  = V(x,5);
    double phi = V(x,6), the = V(x,7), psi = V(x,8);

    double sr = std::sin(phi), cr = std::cos(phi);
    double sp = std::sin(the), cp = std::cos(the), tp = std::tan(the);
    double sy = std::sin(psi), cy = std::cos(psi);

    // Guard singularity
    if (std::abs(cp) < 1e-6) cp = (cp >= 0) ? 1e-6 : -1e-6;

    Mat9 F = Mat9::Identity();

    // ── ∂(ṗ)/∂v = R_bn · dt  ─────────────────────────────────
    Mat3 Rbn = bodyToNav(phi, the, psi);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            F(r, 3+c) = Rbn(r,c) * dt;

    // ── ∂(ṗ)/∂η and ∂(v̇)/∂η ─────────────────────────────────
    // We need ∂R_bn/∂φ, ∂R_bn/∂θ, ∂R_bn/∂ψ applied to v_body and a_body

    // v_body and a_body
    double vb[3] = {vx, vy, vz};
    double ab[3] = {ax, ay, az};

    // ∂R_bn/∂φ (partial wrt roll):
    Mat3 dRdPhi;
    dRdPhi(0,0) =  0;         dRdPhi(0,1) =  cy*sp*cr + sy*sr; dRdPhi(0,2) = -cy*sp*sr + sy*cr;
    dRdPhi(1,0) =  0;         dRdPhi(1,1) =  sy*sp*cr - cy*sr; dRdPhi(1,2) = -sy*sp*sr - cy*cr;
    dRdPhi(2,0) =  0;         dRdPhi(2,1) =  cp*cr;            dRdPhi(2,2) = -cp*sr;

    // ∂R_bn/∂θ (partial wrt pitch):
    Mat3 dRdThe;
    dRdThe(0,0) = -cy*sp;     dRdThe(0,1) =  cy*cp*sr;  dRdThe(0,2) =  cy*cp*cr;
    dRdThe(1,0) = -sy*sp;     dRdThe(1,1) =  sy*cp*sr;  dRdThe(1,2) =  sy*cp*cr;
    dRdThe(2,0) = -cp;        dRdThe(2,1) = -sp*sr;     dRdThe(2,2) = -sp*cr;

    // ∂R_bn/∂ψ (partial wrt yaw):
    Mat3 dRdPsi;
    dRdPsi(0,0) = -sy*cp;     dRdPsi(0,1) = -sy*sp*sr - cy*cr; dRdPsi(0,2) = -sy*sp*cr + cy*sr;
    dRdPsi(1,0) =  cy*cp;     dRdPsi(1,1) =  cy*sp*sr - sy*cr; dRdPsi(1,2) =  cy*sp*cr + sy*sr;
    dRdPsi(2,0) =  0;         dRdPsi(2,1) =  0;                dRdPsi(2,2) =  0;

    Mat3 dRs[3] = {dRdPhi, dRdThe, dRdPsi};

    for (int ang = 0; ang < 3; ++ang) {
        for (int row = 0; row < 3; ++row) {
            double dpdeta = 0, dvdeta = 0;
            for (int k = 0; k < 3; ++k) {
                dpdeta += dRs[ang](row,k) * vb[k];
                dvdeta += dRs[ang](row,k) * ab[k];
            }
            F(row,   6+ang) = dpdeta * dt;   // ∂p/∂η
            F(3+row, 6+ang) = dvdeta * dt;   // ∂v/∂η
        }
    }

    // ── ∂(η̇)/∂η  (Euler kinematics Jacobian) ─────────────────
    // W(φ,θ) · ω, need ∂W/∂φ and ∂W/∂θ

    // ∂W/∂φ applied to ω
    // dW/dphi row 0: [0,  cr*tp, -sr*tp]
    // dW/dphi row 1: [0, -sr,    -cr   ]
    // dW/dphi row 2: [0,  cr/cp, -sr/cp]
    double deta_dphi[3], deta_dthe[3];

    deta_dphi[0] = ( cr*tp*gy - sr*tp*gz);
    deta_dphi[1] = (-sr*gy    - cr*gz);
    deta_dphi[2] = ( cr/cp*gy - sr/cp*gz);

    // ∂W/∂θ applied to ω
    // dW/dthe row 0: [0, sr/cos²θ, cr/cos²θ]
    // dW/dthe row 1: [0, 0,         0        ]
    // dW/dthe row 2: [0, sr*sp/cp², cr*sp/cp²]
    double sec2 = 1.0/(cp*cp);
    deta_dthe[0] = (sr*sec2*gy + cr*sec2*gz);
    deta_dthe[1] = 0.0;
    deta_dthe[2] = (sr*sp*sec2/cp*gy + cr*sp*sec2/cp*gz);

    // Fill Jacobian block rows 6-8, cols 6-8
    Mat3 W = eulerKinMatrix(phi, the);
    // ∂η/∂φ
    F(6,6) = 1.0 + deta_dphi[0]*dt;
    F(7,6) =       deta_dphi[1]*dt;
    F(8,6) =       deta_dphi[2]*dt;
    // ∂η/∂θ
    F(6,7) =       deta_dthe[0]*dt;
    F(7,7) = 1.0 + deta_dthe[1]*dt;
    F(8,7) =       deta_dthe[2]*dt;
    // ∂η/∂ψ: W doesn't depend on ψ, so just identity contribution
    F(6,8) = 0.0; F(7,8) = 0.0; F(8,8) = 1.0;

    return F;
}

// ─────────────────────────────────────────────────────────────
//  Process noise Q(dt)
// ─────────────────────────────────────────────────────────────

Mat9 EKF::buildQ(double dt) const {
    Mat9 Q = Mat9::Zero();
    double qpos  = Q_base_(0,0);
    double qvel  = Q_base_(3,3);
    double qatt  = Q_base_(6,6);
    double dt2   = dt*dt;
    double dt3   = dt2*dt;
    double dt4   = dt2*dt2;

    // Position-velocity coupled process noise (proper continuous-discrete conversion)
    // Uses the standard Van Loan method for coupled position-velocity process noise
    for (int i = 0; i < 3; ++i) {
        Q(i,   i  ) = qvel * dt4 / 4.0;              // Position variance from velocity noise
        Q(i,   i+3) = qvel * dt3 / 2.0;              // Position-velocity covariance
        Q(i+3, i  ) = qvel * dt3 / 2.0;              // Velocity-position covariance
        Q(i+3, i+3) = qvel * dt2;                    // Velocity variance
    }
    // Attitude noise from gyro integration (simple discrete model)
    for (int i = 6; i < 9; ++i)
        Q(i,i) = qatt * dt2;

    return Q;
}

// ─────────────────────────────────────────────────────────────
//  EKF Predict  (Nonlinear + Jacobian)
// ─────────────────────────────────────────────────────────────

void EKF::predict(const IMUData& imu, double dt) {
    if (!initialised_ || dt <= 0.0 || dt > 2.0) return;

    double ax = imu.ax, ay = imu.ay, az = imu.az;
    double gx = imu.gx, gy = imu.gy, gz = imu.gz;

    // Cache for Jacobian
    last_ax_=ax; last_ay_=ay; last_az_=az;
    last_gx_=gx; last_gy_=gy; last_gz_=gz;

    // ── Nonlinear state propagation ───────────────────────────
    x_ = stateTransition(x_, ax, ay, az, gx, gy, gz, dt);

    // ── Jacobian linearisation ────────────────────────────────
    Mat9 F = computeJacobian(x_, ax, ay, az, gx, gy, gz, dt);

    // ── Covariance propagation: P = F·P·Fᵀ + Q ───────────────
    Mat9 Q = buildQ(dt);
    P_ = F * P_ * F.transpose() + Q;
    P_.clampDiagonal(1e-9);  // Prevent numerical underflow
}

// ─────────────────────────────────────────────────────────────
//  Generic templated EKF update step
//  Works for any measurement dimension M.
// ─────────────────────────────────────────────────────────────

template<int M>
void EKF::ekfUpdate(const Matrix<M,1>& z,
                    const Matrix<M,N>& H,
                    const Matrix<M,M>& R)
{
    // Innovation: y = z − H·x
    Matrix<M,1> y = z - H * x_;

    // Innovation covariance: S = H·P·Hᵀ + R
    Matrix<M,M> S = H * P_ * H.transpose() + R;

    // Kalman gain: K = P·Hᵀ·S⁻¹
    Matrix<N,M> K = P_ * H.transpose() * S.inverse();

    // State update: x = x + K·y
    x_ = x_ + K * y;

    // Wrap angles after update
    V(x_,6) = wrapAngle(V(x_,6));
    V(x_,7) = wrapAngle(V(x_,7));
    V(x_,8) = wrapAngle(V(x_,8));

    // Covariance update — Joseph form (numerically stable):
    // P = (I - K·H)·P·(I - K·H)ᵀ + K·R·Kᵀ
    Mat9 IKH = Mat9::Identity() + (K * H) * (-1.0);
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    P_.clampDiagonal(1e-9);
}

// Explicit instantiations to avoid linker errors
template void EKF::ekfUpdate<1>(const Matrix<1,1>&, const Matrix<1,9>&, const Matrix<1,1>&);
template void EKF::ekfUpdate<2>(const Matrix<2,1>&, const Matrix<2,9>&, const Matrix<2,2>&);
template void EKF::ekfUpdate<3>(const Matrix<3,1>&, const Matrix<3,9>&, const Matrix<3,3>&);

// ─────────────────────────────────────────────────────────────
//  Sensor Update Methods
// ─────────────────────────────────────────────────────────────

// ── DVL: measures body-frame velocity [vx, vy, vz] ───────────
//    H_dvl picks state indices 3, 4, 5
void EKF::updateDVL(const DVLData& dvl) {
    if (!initialised_ || !dvl.valid) return;

    Vec3 z; V(z,0)=dvl.vx; V(z,1)=dvl.vy; V(z,2)=dvl.vz;

    Mat3x9 H = Mat3x9::Zero();
    H(0,3) = 1.0;
    H(1,4) = 1.0;
    H(2,5) = 1.0;

    ekfUpdate<3>(z, H, R_dvl_);
}

// ── Depth sensor: measures absolute pz ───────────────────────
//    H_depth picks state index 2
void EKF::updateDepth(const DepthData& depth) {
    if (!initialised_ || !depth.valid) return;

    Vec1 z; V(z,0) = depth.depth;

    Mat1x9 H = Mat1x9::Zero();
    H(0,2) = 1.0;

    ekfUpdate<1>(z, H, R_depth_);
}

// ── GPS/USBL: measures absolute px, py ───────────────────────
//    H_gps picks state indices 0, 1
void EKF::updateGPS(const GPSData& gps) {
    if (!initialised_ || !gps.valid) return;

    Vec2 z; V(z,0)=gps.x; V(z,1)=gps.y;

    Mat2x9 H = Mat2x9::Zero();
    H(0,0) = 1.0;
    H(1,1) = 1.0;

    ekfUpdate<2>(z, H, R_gps_);
}

// ── Magnetometer: measures yaw ────────────────────────────────
//    H_mag picks state index 8
//    IMPORTANT: Innovation computed with angle wrapping
void EKF::updateMag(const MagData& mag) {
    if (!initialised_ || !mag.valid) return;

    Vec1 z; V(z,0) = mag.yaw_rad;

    Mat1x9 H = Mat1x9::Zero();
    H(0,8) = 1.0;

    // Custom innovation with angle wrap
    double predicted_yaw = V(x_, 8);
    double innovation = wrapAngle(mag.yaw_rad - predicted_yaw);

    Vec1 y; V(y,0) = innovation;
    Mat1 S = H * P_ * H.transpose() + R_mag_;
    Mat9x1 K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    V(x_,6) = wrapAngle(V(x_,6));
    V(x_,7) = wrapAngle(V(x_,7));
    V(x_,8) = wrapAngle(V(x_,8));

    Mat9 IKH = Mat9::Identity() + (K * H) * (-1.0);
    P_ = IKH * P_ * IKH.transpose() + K * R_mag_ * K.transpose();
    P_.clampDiagonal(1e-9);
}

// ─────────────────────────────────────────────────────────────
//  NavState assembly
// ─────────────────────────────────────────────────────────────

NavState EKF::getNavState(double timestamp) {
    NavState ns{};
    ns.timestamp = timestamp;

    ns.px    = V(x_,0); ns.py    = V(x_,1); ns.pz    = V(x_,2);
    ns.vx    = V(x_,3); ns.vy    = V(x_,4); ns.vz    = V(x_,5);
    ns.roll  = V(x_,6); ns.pitch = V(x_,7); ns.yaw   = V(x_,8);
    ns.depth = V(x_,2);

    ns.speed_horizontal = std::sqrt(ns.vx*ns.vx + ns.vy*ns.vy);
    ns.speed_total      = std::sqrt(ns.vx*ns.vx + ns.vy*ns.vy + ns.vz*ns.vz);

    double dx = ns.px-prev_px_, dy = ns.py-prev_py_, dz = ns.pz-prev_pz_;
    distance_total_ += std::sqrt(dx*dx + dy*dy + dz*dz);
    prev_px_=ns.px; prev_py_=ns.py; prev_pz_=ns.pz;
    ns.distance_total = distance_total_;

    // Uncertainty (RMS of diagonal covariance per group)
    double cp = (P_(0,0)+P_(1,1)+P_(2,2))/3.0;
    double cv = (P_(3,3)+P_(4,4)+P_(5,5))/3.0;
    double ca = (P_(6,6)+P_(7,7)+P_(8,8))/3.0;
    ns.std_pos_m   = (cp > 0) ? std::sqrt(cp)          : 0;
    ns.std_vel_ms  = (cv > 0) ? std::sqrt(cv)          : 0;
    ns.std_att_deg = (ca > 0) ? std::sqrt(ca)*180.0/M_PI : 0;

    return ns;
}

} 
