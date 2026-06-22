#include "ekf.hpp"
#include "csv_parser.hpp"
#include "logger.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#ifdef _WIN32
#include <direct.h>     // _mkdir on Windows
#else
#include <sys/stat.h>   // ::mkdir on POSIX
#endif

using namespace auv;

// ─────────────────────────────────────────────────────────────
//  Utility: create a directory (ignore if already exists)
// ─────────────────────────────────────────────────────────────
static void ensureDir(const std::string& dir) {
#ifdef _WIN32
    _mkdir(dir.c_str());
#else
    ::mkdir(dir.c_str(), 0755);
#endif
}

// ─────────────────────────────────────────────────────────────
//  Utility: derive directory from a file path and create it
// ─────────────────────────────────────────────────────────────
static void ensureDirForFile(const std::string& filepath) {
    size_t pos = filepath.find_last_of("/\\");
    if (pos != std::string::npos && pos > 0) {
        ensureDir(filepath.substr(0, pos));
    }
}

// ─────────────────────────────────────────────────────────────
//  Print live table
// ─────────────────────────────────────────────────────────────
static void printHeader() {
    std::cout << "\n\033[1;36m"
            << std::setw(8)  << "t(s)"
            << std::setw(9)  << "PX(m)"
            << std::setw(9)  << "PY(m)"
            << std::setw(8)  << "Dep(m)"
            << std::setw(8)  << "Spd"
            << std::setw(9)  << "Dist(m)"
            << std::setw(8)  << "Yaw"
            << std::setw(8)  << "Pitch"
            << std::setw(8)  << "Roll"
            << std::setw(8)  << "sPosm"
            << std::setw(7)  << "Sens"
            << "\033[0m\n"
            << std::string(100, '-') << "\n";
}

static void printRow(const NavState& ns, int& rowcount) {
    if (rowcount % 25 == 0) printHeader();

    // Sensor active this step: D=DVL Z=depth G=GPS M=mag
    char sens[8] = "----";
    if (ns.updated_dvl)   sens[0] = 'D';
    if (ns.updated_depth) sens[1] = 'Z';
    if (ns.updated_gps)   sens[2] = 'G';
    if (ns.updated_mag)   sens[3] = 'M';

    std::cout << std::fixed << std::setprecision(2)
            << std::setw(8)  << ns.timestamp
            << std::setw(9)  << ns.px
            << std::setw(9)  << ns.py
            << std::setw(8)  << ns.depth
            << std::setw(8)  << ns.speed_total
            << std::setw(9)  << ns.distance_total
            << std::setw(8)  << ns.yaw   * 180.0 / M_PI
            << std::setw(8)  << ns.pitch * 180.0 / M_PI
            << std::setw(8)  << ns.roll  * 180.0 / M_PI
            << std::setw(8)  << ns.std_pos_m
            << "  " << sens
            << "\n";

    ++rowcount;
}

// ─────────────────────────────────────────────────────────────
//  Print final summary
// ─────────────────────────────────────────────────────────────
static void printSummary(const std::vector<NavState>& res) {
    if (res.empty()) { Logger::warn("No results to summarise."); return; }

    const NavState& last = res.back();
    double max_spd = 0, max_dep = 0;
    int n_dvl = 0, n_depth = 0, n_gps = 0, n_mag = 0;

    for (const auto& ns : res) {
        max_spd  = std::max(max_spd, ns.speed_total);
        max_dep  = std::max(max_dep, ns.depth);
        if (ns.updated_dvl)   ++n_dvl;
        if (ns.updated_depth) ++n_depth;
        if (ns.updated_gps)   ++n_gps;
        if (ns.updated_mag)   ++n_mag;
    }

    auto deg = [](double r) { return std::to_string(r * 180.0 / M_PI); };

    Logger::section("AUV Navigation Mission Report");

    std::cout << std::fixed << std::setprecision(3);

    Logger::kv("Mission Duration",       std::to_string(last.timestamp) + " s");
    Logger::kv("Total Path Length",      std::to_string(last.distance_total) + " m");

    std::cout << "\n";
    std::cout << " Final Position (N,E,D)\n";
    Logger::kv("   North",            std::to_string(last.px) + " m");
    Logger::kv("   East",             std::to_string(last.py) + " m");
    Logger::kv("   Depth",            std::to_string(last.pz) + " m");

    std::cout << "\n";
    std::cout << " Final Orientation\n";
    Logger::kv("   Roll",                deg(last.roll)  + " deg");
    Logger::kv("   Pitch",               deg(last.pitch) + " deg");
    Logger::kv("   Yaw",                 deg(last.yaw)   + " deg");

    std::cout << "\n";
    std::cout << " Vehicle Performance\n";
    Logger::kv("   Maximum Speed",       std::to_string(max_spd) + " m/s");
    Logger::kv("   Maximum Depth",       std::to_string(max_dep) + " m");

    std::cout << "\n";
    std::cout << " EKF Estimation Quality\n";
    Logger::kv("   Position Sigma",      std::to_string(last.std_pos_m) + " m");
    Logger::kv("   Velocity Sigma",      std::to_string(last.std_vel_ms) + " m/s");
    Logger::kv("   Attitude Sigma",      std::to_string(last.std_att_deg) + " deg");

    std::cout << "\n";
    std::cout << " Sensor Statistics\n";
    Logger::kv("   IMU Samples",         std::to_string(res.size()));
    Logger::kv("   DVL Updates",         std::to_string(n_dvl));
    Logger::kv("   Depth Updates",       std::to_string(n_depth));
    Logger::kv("   GPS Updates",         std::to_string(n_gps));
    Logger::kv("   Magnetometer Updates",std::to_string(n_mag));

    std::cout << "\n";
}

// ─────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {

    Logger::section("AUV ");
    Logger::info("State vector : [px  py  pz  |  vx  vy  vz  |  roll  pitch  yaw]");
    Logger::info("Sensors      : IMU + DVL + Depth + GPS/USBL + Magnetometer");

    // ── Default file paths ────────────────────────────────────
    std::string imu_f = "data/imu.csv";
    std::string dvl_f = "data/dvl.csv";
    std::string dep_f = "data/depth.csv";
    std::string gps_f = "data/gps.csv";
    std::string mag_f = "data/mag.csv";
    std::string out_f = "results/nav_output.csv";

    bool gen_data = false;
    int file_idx = 0;
    for (int i = 1; i < argc; ++i) {
        std::string a(argv[i]);
        if      (a == "--generate-data") { gen_data = true; }
        else if (a == "--verbose")       { Logger::setLevel(LogLevel::DEBUG); }
        else {
            // Positional file arguments in order
            switch (file_idx++) {
                case 0: imu_f = a; break;
                case 1: dvl_f = a; break;
                case 2: dep_f = a; break;
                case 3: gps_f = a; break;
                case 4: mag_f = a; break;
                case 5: out_f = a; break;
                default: Logger::warn("Extra argument ignored: " + a);
            }
        }
    }

    //Ensure Data and result dir
    ensureDir("data");
    ensureDir("results");
    ensureDirForFile(out_f);   // handles custom output paths too

    Logger::info("Input  folder: data/");
    Logger::info("Output file  : " + out_f);

    // ── Generate synthetic data if requested ──────────────────
    if (gen_data) {
        Logger::section("Generating Synthetic 5-Sensor Dataset");
        CSVParser::generateSyntheticData(
            "data/imu.csv", "data/dvl.csv", "data/depth.csv",
            "data/gps.csv", "data/mag.csv",
            /*duration*/ 80.0,
            /*dt_imu*/   0.01,
            /*dt_dvl*/   0.1,
            /*dt_depth*/ 0.5,
            /*dt_gps*/   1.0,
            /*dt_mag*/   0.2);
        // Reset to generated paths
        imu_f = "data/imu.csv"; dvl_f = "data/dvl.csv";
        dep_f = "data/depth.csv"; gps_f = "data/gps.csv";
        mag_f = "data/mag.csv";
    }

    // ── Load sensor data ──────────────────────────────────────
    Logger::section("Loading Sensor Data");

    std::vector<IMUData>   imu_data;
    std::vector<DVLData>   dvl_data;
    std::vector<DepthData> dep_data;
    std::vector<GPSData>   gps_data;
    std::vector<MagData>   mag_data;

    try {
        imu_data = CSVParser::loadIMU  (imu_f);
        dvl_data = CSVParser::loadDVL  (dvl_f);
        dep_data = CSVParser::loadDepth(dep_f);
        gps_data = CSVParser::loadGPS  (gps_f);
        mag_data = CSVParser::loadMag  (mag_f);
    } catch (const std::exception& e) {
        Logger::error(std::string(e.what()));
        Logger::warn("Tip: run  ./auv_nav --generate-data  to create test data.");
        return 1;
    }

    if (imu_data.empty()) {
        Logger::error("IMU file has no valid data rows. Cannot run EKF.");
        return 1;
    }

    // ── EKF configuration ─────────────────────────────────────
    // All values are std-deviations (σ) matching your sensor specs.
    // Increase a value → trust that sensor LESS.
    // Decrease a value → trust that sensor MORE.
    EKF ekf(
        /*sigma_acc  */ 0.02,   // IMU accelerometer  [m/s²] - reduced for better integration
        /*sigma_gyro */ 0.001,  // IMU gyroscope      [rad/s] - reduced for smoother attitude
        /*sigma_dvl  */ 0.02,   // DVL velocity       [m/s] - reduced, DVL is very accurate
        /*sigma_depth*/ 0.01,   // Depth sensor       [m] - reduced, pressure sensors are precise
        /*sigma_gps  */ 0.5,    // GPS/USBL position  [m] - much lower for USBL underwater
        /*sigma_mag  */ 0.015,  // Magnetometer yaw   [rad] - slightly reduced
        /*sigma_proc */ 0.01    // Process model noise - significantly reduced to prevent drift
    );

    // ── Initialise EKF from first readings ────────────────────
    Logger::section("Starting EKF Fusion");

    DVLData dvl_init{0.0, 0.0, 0.0, 0.0, false};
    if (!dvl_data.empty()) dvl_init = dvl_data[0];

    double init_yaw = mag_data.empty() ? 0.0 : mag_data[0].yaw_rad;
    ekf.initialise(imu_data[0], dvl_init, 0.0, init_yaw);

    // ── Fusion loop ───────────────────────────────────────────
    std::vector<NavState> results;
    results.reserve(imu_data.size());

    size_t i_dvl = 0, i_dep = 0, i_gps = 0, i_mag = 0;
    int rowcount = 0;

    // Print one table row roughly every 250 console lines
    const int print_every = std::max(1, (int)(imu_data.size() / 250));

    auto wall_t0 = std::chrono::high_resolution_clock::now();

    for (size_t i = 1; i < imu_data.size(); ++i) {

        const IMUData& imu = imu_data[i];
        double dt = imu.timestamp - imu_data[i - 1].timestamp;

        // Sanity-check dt — skip bad timestamps
        if (dt <= 0.0 || dt > 1.0) continue;

        // ── 1. PREDICT — IMU propagates state forward ──────────
        ekf.predict(imu, dt);

        bool u_dvl = false, u_dep = false, u_gps = false, u_mag = false;

        // ── 2. UPDATE: DVL (velocity) ──────────────────────────
        while (i_dvl < dvl_data.size() && dvl_data[i_dvl].timestamp <= imu.timestamp) {
            ekf.updateDVL(dvl_data[i_dvl]);
            if (dvl_data[i_dvl].valid) u_dvl = true;
            ++i_dvl;
        }

        // ── 3. UPDATE: Depth (pz) ──────────────────────────────
        while (i_dep < dep_data.size() && dep_data[i_dep].timestamp <= imu.timestamp) {
            ekf.updateDepth(dep_data[i_dep]);
            if (dep_data[i_dep].valid) u_dep = true;
            ++i_dep;
        }

        // ── 4. UPDATE: GPS/USBL (px, py) ──────────────────────
        while (i_gps < gps_data.size() && gps_data[i_gps].timestamp <= imu.timestamp) {
            ekf.updateGPS(gps_data[i_gps]);
            if (gps_data[i_gps].valid) u_gps = true;
            ++i_gps;
        }

        // ── 5. UPDATE: Magnetometer (yaw) ─────────────────────
        while (i_mag < mag_data.size() && mag_data[i_mag].timestamp <= imu.timestamp) {
            ekf.updateMag(mag_data[i_mag]);
            if (mag_data[i_mag].valid) u_mag = true;
            ++i_mag;
        }

        // ── Record result ──────────────────────────────────────
        NavState ns       = ekf.getNavState(imu.timestamp);
        ns.updated_imu    = true;
        ns.updated_dvl    = u_dvl;
        ns.updated_depth  = u_dep;
        ns.updated_gps    = u_gps;
        ns.updated_mag    = u_mag;
        results.push_back(ns);

        // ── Console output (sampled so terminal isn't flooded) ─
        if ((int)i % print_every == 0) {
            printRow(ns, rowcount);
        }
    }

    auto wall_t1 = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(wall_t1 - wall_t0).count();
    double rate = (elapsed_ms > 0) ? (results.size() / (elapsed_ms / 1000.0)) : 0;

    Logger::info(
        "Fusion complete — " + std::to_string(results.size())
        + " samples in " + std::to_string((int)elapsed_ms)
        + " ms  (" + std::to_string((int)rate) + " samples/s)"
    );

    // ── Save results to CSV ───────────────────────────────────
    Logger::section("Saving Results");

    if (results.empty()) {
        Logger::error("No results computed — check your input CSV files.");
        return 1;
    }

    try {
        CSVParser::saveResults(results, out_f);
    } catch (const std::exception& e) {
        Logger::error("Save failed: " + std::string(e.what()));
        return 1;
    }

    // ── Print summary ─────────────────────────────────────────
    printSummary(results);

    Logger::info("Done. Open  " + out_f + "  to view results.");
    return 0;
}
