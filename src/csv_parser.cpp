#include "csv_parser.hpp"
#include "logger.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <random>
#include <iomanip>
#include <stdexcept>
#include <cerrno>
#include <cstring>
#ifdef _WIN32
#include <direct.h>     // _mkdir on Windows
#define MKDIR(path) _mkdir(path)
#else
#include <sys/stat.h>   // mkdir on POSIX
#define MKDIR(path) ::mkdir(path, 0755)
#endif

namespace auv {

// ─────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────

std::string CSVParser::trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    size_t b = s.find_last_not_of (" \t\r\n");
    return (a == std::string::npos) ? "" : s.substr(a, b-a+1);
}

std::vector<std::string> CSVParser::split(const std::string& line, char d) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, d)) out.push_back(trim(tok));
    return out;
}

// Skip header (first row that doesn't parse as a number)
static bool tryParseRow(const std::vector<std::string>& toks, int n) {
    if ((int)toks.size() < n) return false;
    try { std::stod(toks[0]); return true; }
    catch (...) { return false; }
}

// ─────────────────────────────────────────────────────────────
//  Loaders
// ─────────────────────────────────────────────────────────────

std::vector<IMUData> CSVParser::loadIMU(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open IMU: " + path);
    std::vector<IMUData> out;
    std::string line;
    while (std::getline(f, line)) {
        line = trim(line);
        if (line.empty() || line[0]=='#') continue;
        auto t = split(line);
        if (!tryParseRow(t, 7)) continue;
        try {
            IMUData d;
            d.timestamp = std::stod(t[0]);
            d.ax=std::stod(t[1]); d.ay=std::stod(t[2]); d.az=std::stod(t[3]);
            d.gx=std::stod(t[4]); d.gy=std::stod(t[5]); d.gz=std::stod(t[6]);
            out.push_back(d);
        } catch(...) {}
    }
    std::sort(out.begin(),out.end(),[](auto&a,auto&b){return a.timestamp<b.timestamp;});
    Logger::info("IMU   : " + std::to_string(out.size()) + " records ← " + path);
    return out;
}

std::vector<DVLData> CSVParser::loadDVL(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open DVL: " + path);
    std::vector<DVLData> out;
    std::string line;
    while (std::getline(f, line)) {
        line = trim(line);
        if (line.empty() || line[0]=='#') continue;
        auto t = split(line);
        if (!tryParseRow(t, 5)) continue;
        try {
            DVLData d;
            d.timestamp=std::stod(t[0]);
            d.vx=std::stod(t[1]); d.vy=std::stod(t[2]); d.vz=std::stod(t[3]);
            d.valid=(std::stoi(t[4])!=0);
            out.push_back(d);
        } catch(...) {}
    }
    std::sort(out.begin(),out.end(),[](auto&a,auto&b){return a.timestamp<b.timestamp;});
    Logger::info("DVL   : " + std::to_string(out.size()) + " records ← " + path);
    return out;
}

std::vector<DepthData> CSVParser::loadDepth(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open Depth: " + path);
    std::vector<DepthData> out;
    std::string line;
    while (std::getline(f, line)) {
        line = trim(line);
        if (line.empty() || line[0]=='#') continue;
        auto t = split(line);
        if (!tryParseRow(t, 3)) continue;
        try {
            DepthData d;
            d.timestamp=std::stod(t[0]); d.depth=std::stod(t[1]);
            d.valid=(std::stoi(t[2])!=0);
            out.push_back(d);
        } catch(...) {}
    }
    std::sort(out.begin(),out.end(),[](auto&a,auto&b){return a.timestamp<b.timestamp;});
    Logger::info("Depth : " + std::to_string(out.size()) + " records ← " + path);
    return out;
}

std::vector<GPSData> CSVParser::loadGPS(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open GPS: " + path);
    std::vector<GPSData> out;
    std::string line;
    while (std::getline(f, line)) {
        line = trim(line);
        if (line.empty() || line[0]=='#') continue;
        auto t = split(line);
        if (!tryParseRow(t, 4)) continue;
        try {
            GPSData d;
            d.timestamp=std::stod(t[0]); d.x=std::stod(t[1]); d.y=std::stod(t[2]);
            d.valid=(std::stoi(t[3])!=0);
            out.push_back(d);
        } catch(...) {}
    }
    std::sort(out.begin(),out.end(),[](auto&a,auto&b){return a.timestamp<b.timestamp;});
    Logger::info("GPS   : " + std::to_string(out.size()) + " records ← " + path);
    return out;
}

std::vector<MagData> CSVParser::loadMag(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open Mag: " + path);
    std::vector<MagData> out;
    std::string line;
    while (std::getline(f, line)) {
        line = trim(line);
        if (line.empty() || line[0]=='#') continue;
        auto t = split(line);
        if (!tryParseRow(t, 3)) continue;
        try {
            MagData d;
            d.timestamp=std::stod(t[0]); d.yaw_rad=std::stod(t[1]);
            d.valid=(std::stoi(t[2])!=0);
            out.push_back(d);
        } catch(...) {}
    }
    std::sort(out.begin(),out.end(),[](auto&a,auto&b){return a.timestamp<b.timestamp;});
    Logger::info("Mag   : " + std::to_string(out.size()) + " records ← " + path);
    return out;
}

// ─────────────────────────────────────────────────────────────
//  Save results
// ─────────────────────────────────────────────────────────────

// ── Helper: create every directory in a path (like mkdir -p) ─
static void mkdirP(const std::string& path) {
    // Find the last '/' to get the directory part
    size_t pos = path.find_last_of("/\\");
    if (pos == std::string::npos) return;   // no directory part
    std::string dir = path.substr(0, pos);
    if (dir.empty()) return;

    // Walk forward and create each component
    std::string cur;
    for (size_t i = 0; i < dir.size(); ++i) {
        char c = dir[i];
        if (c == '/' || c == '\\') {
            if (!cur.empty())
                MKDIR(cur.c_str());   // ignore errors (already exists)
        }
        cur += c;
    }
    MKDIR(cur.c_str());
}

void CSVParser::saveResults(const std::vector<NavState>& results, const std::string& path) {

    // ── Step 1: Create output directory if it doesn't exist ───
    mkdirP(path);
    Logger::info("Output path   : " + path);

    // ── Step 2: Open file ─────────────────────────────────────
    std::ofstream f(path, std::ios::out | std::ios::trunc);
    if (!f.is_open()) {
        throw std::runtime_error(
            "Cannot open output file: " + path +
            "\n  Reason: " + std::string(std::strerror(errno)) +
            "\n  Fix: make sure the 'results' folder exists next to your binary.\n"
            "  Run:  mkdir -p results");
    }

    // ── Step 3: Write header ──────────────────────────────────
    f<< "timestamp,"
    << "px_m,py_m,pz_m,"
    << "vx_ms,vy_ms,vz_ms,"
    << "roll_deg,pitch_deg,yaw_deg,"
    << "speed_horiz_ms,speed_total_ms,"
    << "distance_total_m,depth_m,"
    << "std_pos_m,std_vel_ms,std_att_deg,"
    << "upd_imu,upd_dvl,upd_depth,upd_gps,upd_mag\n";

    if (!f.good()) throw std::runtime_error("Failed writing header to: " + path);

    // ── Step 4: Write data rows ───────────────────────────────
    f << std::fixed << std::setprecision(6);
    for (const auto& ns : results) {
        f << ns.timestamp << ","
        << ns.px << "," << ns.py << "," << ns.pz << ","
        << ns.vx << "," << ns.vy << "," << ns.vz << ","
        << ns.roll  * 180.0/M_PI << ","
        << ns.pitch * 180.0/M_PI << ","
        << ns.yaw   * 180.0/M_PI << ","
        << ns.speed_horizontal << ","
        << ns.speed_total << ","
        << ns.distance_total << ","
        << ns.depth << ","
        << ns.std_pos_m << ","
        << ns.std_vel_ms << ","
        << ns.std_att_deg << ","
        << ns.updated_imu   << ","
        << ns.updated_dvl   << ","
        << ns.updated_depth << ","
        << ns.updated_gps   << ","
        << ns.updated_mag   << "\n";
    }

    // ── Step 5: Flush + close, then verify ───────────────────
    f.flush();
    if (!f.good()) throw std::runtime_error("Write error (disk full?) for: " + path);
    f.close();

    // ── Step 6: Confirm file exists and has data ──────────────
    std::ifstream check(path, std::ios::ate | std::ios::binary);
    if (!check.is_open()) {
        throw std::runtime_error("File was not created on disk: " + path);
    }
    std::streamsize bytes = check.tellg();
    check.close();

    Logger::info("Results saved : " + std::to_string(results.size()) + " rows  |  " + std::to_string(bytes / 1024) + " KB  →  " + path);
}

// ─────────────────────────────────────────────────────────────
//  Synthetic mission generator
//  ──────────────────────────────────────────────────────────
//  True vehicle dynamics simulated with Euler integration.
//  Sensor readings = true value + Gaussian noise.
//  GPS goes invalid when depth > 2 m (submerged).
// ─────────────────────────────────────────────────────────────

void CSVParser::generateSyntheticData(
    const std::string& imu_out, const std::string& dvl_out,
    const std::string& depth_out, const std::string& gps_out,
    const std::string& mag_out,
    double duration, double dt_imu, double dt_dvl,
    double dt_depth, double dt_gps, double dt_mag)
{
    std::mt19937 rng(2024);
    auto nrm = [&](double s) {
        return std::normal_distribution<double>(0,s)(rng);
    };

    // Auto-create the data/ folder before writing
    mkdirP(imu_out);
    mkdirP(dvl_out);
    mkdirP(depth_out);
    mkdirP(gps_out);
    mkdirP(mag_out);
    std::ofstream fi(imu_out), fd(dvl_out), fz(depth_out), fg(gps_out), fm(mag_out);
    if (!fi||!fd||!fz||!fg||!fm)
        throw std::runtime_error("Cannot create synthetic data files");

    fi << "# AUV Synthetic IMU  — timestamp,ax,ay,az,gx,gy,gz\n";
    fi << "timestamp,ax,ay,az,gx,gy,gz\n";
    fd << "# AUV Synthetic DVL  — timestamp,vx,vy,vz,valid\n";
    fd << "timestamp,vx,vy,vz,valid\n";
    fz << "# AUV Synthetic Depth — timestamp,depth_m,valid\n";
    fz << "timestamp,depth_m,valid\n";
    fg << "# AUV Synthetic GPS  — timestamp,x_m,y_m,valid\n";
    fg << "timestamp,x_m,y_m,valid\n";
    fm << "# AUV Synthetic Mag  — timestamp,yaw_rad,valid\n";
    fm << "timestamp,yaw_rad,valid\n";

    fi << std::fixed << std::setprecision(7);
    fd << std::fixed << std::setprecision(7);
    fz << std::fixed << std::setprecision(5);
    fg << std::fixed << std::setprecision(5);
    fm << std::fixed << std::setprecision(7);

    // True state
    double px=0, py=0, pz=0;
    double vx=0, vy=0, vz=0;
    double roll=0, pitch=0, yaw=0.3;  // start heading 0.3 rad

    // True inputs (body frame)
    double ax_b=0, ay_b=0, az_b=0;
    double gx_b=0, gy_b=0, gz_b=0;

    static constexpr double G = 9.81;

    double t = 0.0;
    double next_dvl=0, next_depth=0, next_gps=0, next_mag=0;

    auto Rbn = [](double r, double p, double y_) -> std::array<std::array<double,3>,3> {
        double cr=cos(r),sr=sin(r),cp=cos(p),sp=sin(p),cy=cos(y_),sy=sin(y_);
        return {{
            {cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr},
            {sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr},
            {-sp,   cp*sr,          cp*cr}}};
    };

    while (t <= duration) {

        // ── Define true body-frame accelerations per mission phase ──
        if (t < 20.0) {
            // Descent + surge: pitch down 10°, accelerate forward
            double phase = t / 20.0;
            double target_pitch = -0.18;   // -10° (nose down)
            pitch += (target_pitch - pitch) * 0.005;
            ax_b = 0.4 * (1.0 - phase * 0.5);  // surge fwd
            ay_b = 0.0;
            az_b = 0.0;
            gx_b = 0.0; gy_b = -0.008; gz_b = 0.0;  // pitch-down rate

        } else if (t < 40.0) {
            // Banked turn: roll 20°, yaw rate
            double target_roll = 0.35;   // 20° bank
            roll += (target_roll - roll) * 0.01;
            ax_b = 0.05;
            ay_b = 0.0;
            az_b = 0.0;
            gx_b = 0.005; gy_b = 0.0; gz_b = 0.04;  // yaw rate

        } else if (t < 60.0) {
            // Level cruise: roll back to 0, pitch to 0
            roll  += (0.0 - roll)  * 0.01;
            pitch += (0.0 - pitch) * 0.01;
            ax_b  = 0.0; ay_b = 0.0; az_b = 0.0;
            gx_b  = 0.0; gy_b = 0.0; gz_b = 0.0;

        } else {
            // Ascent: pitch up, decelerate
            double target_pitch = 0.15;
            pitch += (target_pitch - pitch) * 0.005;
            ax_b = -0.3 * (t-60.0)/20.0;
            ay_b = 0.0; az_b = 0.0;
            gx_b = 0.0; gy_b = 0.008; gz_b = 0.0;
        }

        // ── True kinematics integration ───────────────────────
        auto R = Rbn(roll, pitch, yaw);

        // Velocity update: v += (R·a_body - g_nav) · dt
        double ax_n = R[0][0]*ax_b + R[0][1]*ay_b + R[0][2]*az_b;
        double ay_n = R[1][0]*ax_b + R[1][1]*ay_b + R[1][2]*az_b;
        double az_n = R[2][0]*ax_b + R[2][1]*ay_b + R[2][2]*az_b - G;

        vx += ax_n * dt_imu;
        vy += ay_n * dt_imu;
        vz += az_n * dt_imu;

        // Position update: p += R·v_body · dt
        double vbx = vx, vby = vy, vbz = vz;
        px += (R[0][0]*vbx + R[0][1]*vby + R[0][2]*vbz) * dt_imu;
        py += (R[1][0]*vbx + R[1][1]*vby + R[1][2]*vbz) * dt_imu;
        pz += (R[2][0]*vbx + R[2][1]*vby + R[2][2]*vbz) * dt_imu;

        // px += vx * dt_imu;
        // py += vy * dt_imu;
        // pz += vz * dt_imu;

        // Attitude update (Euler): η += W·ω · dt
        double tp = tan(pitch), cp2 = cos(pitch);
        if (fabs(cp2) < 1e-6) cp2 = 1e-6;
        double sr=sin(roll), cr=cos(roll);
        double dphi = gx_b + sr*tp*gy_b  + cr*tp*gz_b;
        double dthe =        cr*gy_b     - sr*gz_b;
        double dpsi =        sr/cp2*gy_b + cr/cp2*gz_b;
        roll  = fmod(roll  + dphi*dt_imu + M_PI, 2*M_PI) - M_PI;
        pitch = fmod(pitch + dthe*dt_imu + M_PI, 2*M_PI) - M_PI;
        yaw   = fmod(yaw   + dpsi*dt_imu + M_PI, 2*M_PI) - M_PI;

        // Clamp depth (can't go above surface)
        if (pz < 0) { pz = 0; vz = 0; }

        // ── IMU measurement (accel = R_bn^T · (a_nav + g) in body frame) ─
        // What IMU measures: a_body + noise (gravity is not subtracted by IMU)
        double ax_imu = ax_b + G*sin(pitch)        + nrm(0.05);
        double ay_imu = ay_b - G*cos(pitch)*sin(roll) + nrm(0.05);
        double az_imu = az_b - G*cos(pitch)*cos(roll) + nrm(0.05);

        fi << t << ","
            << ax_imu       << "," << ay_imu       << "," << az_imu << ","
            << gx_b+nrm(0.003) << "," << gy_b+nrm(0.003) << "," << gz_b+nrm(0.003) << "\n";

        // ── DVL @ lower rate ──────────────────────────────────
        // if (t >= next_dvl) {
        //     bool valid = std::uniform_real_distribution<double>(0,1)(rng) > 0.05;
        //     fd << t << ","
        //         << vx+nrm(0.02) << "," << vy+nrm(0.02) << "," << vz+nrm(0.02) << ","
        //         << (valid?1:0) << "\n";
        //     next_dvl += dt_dvl;
        // }

        // DVL @ lower rate (body-frame velocity)
        if (t >= next_dvl) {

            bool valid =
                std::uniform_real_distribution<double>(0,1)(rng) > 0.05;

            // Navigation velocity -> Body velocity
            double vbx =
                R[0][0]*vx + R[1][0]*vy + R[2][0]*vz;

            double vby =
                R[0][1]*vx + R[1][1]*vy + R[2][1]*vz;

            double vbz =
                R[0][2]*vx + R[1][2]*vy + R[2][2]*vz;

            fd << t << ","
            << vbx + nrm(0.02) << ","
            << vby + nrm(0.02) << ","
            << vbz + nrm(0.02) << ","
            << (valid ? 1 : 0) << "\n";

            next_dvl += dt_dvl;
        }

        // ── Depth @ lower rate ────────────────────────────────
        if (t >= next_depth) {
            // fz << t << "," << pz+nrm(0.05) << ",1\n";
            double depth_meas =
            std::max(0.0, pz + nrm(0.05));

            fz << t << ","
            << depth_meas
            << ",1\n";
            next_depth += dt_depth;
        }

        // ── GPS: only near surface (pz < 2 m) ─────────────────
        if (t >= next_gps) {
            bool gps_valid = (pz < 2.0);
            fg << t << ","
                << px+nrm(1.5) << "," << py+nrm(1.5) << ","
                << (gps_valid?1:0) << "\n";
            next_gps += dt_gps;
        }

        // ── Magnetometer ──────────────────────────────────────
        if (t >= next_mag) {
            fm << t << "," << yaw+nrm(0.02) << ",1\n";
            next_mag += dt_mag;
        }

        t += dt_imu;
    }

    Logger::info("Synthetic data generated:");
    Logger::info("  IMU   → " + imu_out);
    Logger::info("  DVL   → " + dvl_out);
    Logger::info("  Depth → " + depth_out);
    Logger::info("  GPS   → " + gps_out);
    Logger::info("  Mag   → " + mag_out);
}

}