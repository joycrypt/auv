#pragma once
/**
 * ============================================================
 *  Purpose : Extended Kalman Filter — 9-DOF state estimation
 *            with 5-sensor fusion
 *
 * ┌─────────────────────────────────────────────────────────┐
 * │  STATE VECTOR  x ∈ ℝ⁹                                  │
 * ├───────┬───────────┬──────────────────────────────────── │
 * │ Index │  State    │  Description                        │
 * ├───────┼───────────┼──────────────────────────────────── │
 * │  0    │  px       │  Position X (North)         [m]     │
 * │  1    │  py       │  Position Y (East)          [m]     │
 * │  2    │  pz       │  Position Z (Down, +ve=deep)[m]     │
 * │  3    │  vx       │  Velocity X (body/NED)      [m/s]  │
 * │  4    │  vy       │  Velocity Y                 [m/s]  │
 * │  5    │  vz       │  Velocity Z                 [m/s]  │
 * │  6    │  roll     │  Roll angle    φ            [rad]   │
 * │  7    │  pitch    │  Pitch angle   θ            [rad]   │
 * │  8    │  yaw      │  Yaw angle     ψ            [rad]   │
 * └───────┴───────────┴──────────────────────────────────── ┘
 *
 * ┌────────────────────────────────────────────────────────────┐
 * │  SENSOR SUITE                                              │
 * ├──────────────────┬─────────────────────────────────────── │
 * │  IMU accel       │  Provides body-frame acceleration       │
 * │                  │  → predict velocity + attitude          │
 * │  IMU gyro        │  Provides body-frame angular rate       │
 * │                  │  → propagate roll/pitch/yaw via Euler   │
 * │  DVL             │  Provides body-frame velocity           │
 * │                  │  → update vx, vy, vz                   │
 * │  Depth           │  Provides absolute depth (pressure)     │
 * │                  │  → update pz directly                   │
 * │  GPS / USBL      │  Provides absolute x-y position         │
 * │                  │  → update px, py                        │
 * │  Magnetometer    │  Provides heading (yaw)                 │
 * │                  │  → update yaw with declination corr.    │
 * └──────────────────┴─────────────────────────────────────── ┘
 *
 *  NONLINEARITY:
 *    The state transition is nonlinear because attitude kinematics
 *    use trig (sin/cos of roll/pitch/yaw). We linearise by computing
 *    the Jacobian F = ∂f/∂x at each step (first-order EKF).
 *
 *  ANGLE WRAPPING:
 *    After every update, yaw is wrapped to [-π, π].
 *
 *  NUMERICAL STABILITY:
 *    Covariance update uses the Joseph form:
 *      P = (I-KH)P(I-KH)ᵀ + KRKᵀ
 *    to guarantee P remains symmetric positive-definite.
 * ============================================================
 */

#include "matrix.hpp"
#include <string>

namespace auv {

// ─────────────────────────────────────────────────────────────
//  Sensor data structures
// ─────────────────────────────────────────────────────────────

struct IMUData {
    double timestamp;
    double ax, ay, az;    // Accelerometer  [m/s²]  body frame
    double gx, gy, gz;    // Gyroscope      [rad/s] body frame
};

struct DVLData {
    double timestamp;
    double vx, vy, vz;    // Velocity       [m/s]   body frame
    bool   valid;          // False on beam dropout — update skipped
};

struct DepthData {
    double timestamp;
    double depth;          // Depth below surface [m], positive down
    bool   valid;
};

struct GPSData {
    double timestamp;
    double x, y;           // Position in local tangent plane [m]
    bool   valid;          // False when submerged / no fix
};

struct MagData {
    double timestamp;
    double yaw_rad;        // Magnetic heading corrected for declination [rad]
    bool   valid;
};

// ─────────────────────────────────────────────────────────────
//  Navigation result per time step
// ─────────────────────────────────────────────────────────────

struct NavState {
    double timestamp;

    // Position
    double px, py, pz;         // [m]
    // Velocity
    double vx, vy, vz;         // [m/s]
    // Attitude
    double roll, pitch, yaw;   // [rad]

    // Derived quantities
    double speed_horizontal;   // √(vx²+vy²)        [m/s]
    double speed_total;        // √(vx²+vy²+vz²)    [m/s]
    double distance_total;     // Cumulative path    [m]
    double depth;              // = pz               [m]

    // Uncertainty (diagonal of P, one per state group)
    double std_pos_m;          // √(P[0,0]+P[1,1]+P[2,2]) / 3  [m]
    double std_vel_ms;         // √(P[3,3]+P[4,4]+P[5,5]) / 3  [m/s]
    double std_att_deg;        // √(P[6,6]+P[7,7]+P[8,8]) / 3  [deg]

    // Which sensors were active this step
    bool updated_imu, updated_dvl, updated_depth, updated_gps, updated_mag;
};

// ─────────────────────────────────────────────────────────────
//  EKF class
// ─────────────────────────────────────────────────────────────

class EKF {
public:
    // Dimensions
    static constexpr int N = 9;   // State dimension

    /**
     * @param sigma_acc   Accelerometer noise std  [m/s²]
     * @param sigma_gyro  Gyroscope noise std       [rad/s]
     * @param sigma_dvl   DVL velocity noise std    [m/s]
     * @param sigma_depth Depth sensor noise std    [m]
     * @param sigma_gps   GPS/USBL position std     [m]
     * @param sigma_mag   Magnetometer heading std  [rad]
     * @param sigma_proc  Process model noise std
     **/
    EKF(double sigma_acc   = 0.05,
        double sigma_gyro  = 0.003,
        double sigma_dvl   = 0.05,
        double sigma_depth = 0.05,
        double sigma_gps   = 1.5,
        double sigma_mag   = 0.02,
        double sigma_proc  = 0.05
    );

    /** Set initial state from first sensor readings */
    void initialise(const IMUData& imu,
                    const DVLData& dvl,
                    double initial_depth = 0.0,
                    double initial_yaw_rad = 0.0);

    /**
     * @brief Predict step — nonlinear kinematics + Jacobian linearisation
     * @param imu  IMU reading (both accel AND gyro used here)
     * @param dt   Time since last predict [s]
     */
    void predict(const IMUData& imu, double dt);

    /** Update: DVL body-frame velocity [vx vy vz] → state indices 3,4,5 */
    void updateDVL(const DVLData& dvl);

    /** Update: Depth sensor absolute pz → state index 2 */
    void updateDepth(const DepthData& depth);

    /** Update: GPS/USBL absolute px, py → state indices 0,1 */
    void updateGPS(const GPSData& gps);

    /** Update: Magnetometer yaw → state index 8 (with wrap handling) */
    void updateMag(const MagData& mag);

    /** Assemble and return the current NavState */
    NavState getNavState(double timestamp);

    bool isInitialised() const { return initialised_; }

    // Raw access for debugging / logging
    const Vec9& stateVec()   const { return x_; }
    const Mat9& covarianceMat() const { return P_; }

private:
    Vec9 x_;          // State estimate
    Mat9 P_;          // Covariance matrix

    // Noise matrices
    Mat9  Q_base_;    // Base process noise (scaled by dt inside predict)
    Mat3  R_acc_;     // Accelerometer noise (used inside predict for accel-only update path)
    Mat3  R_dvl_;     // DVL measurement noise
    Mat1  R_depth_;   // Depth measurement noise
    Mat2  R_gps_;     // GPS measurement noise
    Mat1  R_mag_;     // Magnetometer noise

    // Tracking
    double prev_px_, prev_py_, prev_pz_;
    double distance_total_;
    bool   initialised_;

    // Last gyro + accel for Jacobian
    double last_gx_, last_gy_, last_gz_;
    double last_ax_, last_ay_, last_az_;

    // ── Internal helpers ──────────────────────────────────────

    /**
     * Nonlinear state transition f(x, u, dt):
     *   Position   : p += R_body2nav · v · dt
     *   Velocity   : v += (R_body2nav · a_body - g_nav) · dt
     *   Attitude   : η += W(η) · ω_body · dt   (Euler angle kinematics)
     *
     * Where R_body2nav is the 3×3 rotation matrix from body to navigation frame.
     */
    Vec9 stateTransition(const Vec9& x,
                        double ax, double ay, double az,
                        double gx, double gy, double gz,
                        double dt) const;

    /**
     * Jacobian F = ∂f/∂x  (9×9), evaluated at current x.
     * Needed because f is nonlinear in attitude angles.
     */
    Mat9 computeJacobian(const Vec9& x,
                        double ax, double ay, double az,
                        double gx, double gy, double gz,
                        double dt) const;

    /** Build process noise Q scaled properly for dt */
    Mat9 buildQ(double dt) const;

    /**
     * Generic EKF update step (works for any measurement dimension M).
     * Templated to avoid code duplication across all 5 sensors.
     *
     * @tparam M  Measurement dimension
     * @param  z  Measurement vector
     * @param  H  Observation matrix (M×9)
     * @param  R  Measurement noise (M×M)
     */
    template<int M>
    void ekfUpdate(const Matrix<M,1>& z,
                    const Matrix<M,N>& H,
                    const Matrix<M,M>& R);

    /** Wrap angle to [-π, π] */
    static double wrapAngle(double a);

    /** Body-to-navigation rotation matrix from roll, pitch, yaw */
    static Mat3 bodyToNav(double roll, double pitch, double yaw);

    /** Euler angle kinematics matrix W(η) such that η̇ = W·ω */
    static Mat3 eulerKinMatrix(double roll, double pitch);
};

} 
