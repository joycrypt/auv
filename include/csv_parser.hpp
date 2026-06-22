#pragma once

#include "ekf.hpp"

#include <string>
#include <vector>

namespace auv
{

class CSVParser
{
public:
    static std::vector<IMUData> loadIMU(const std::string& path);

    static std::vector<DVLData> loadDVL(const std::string& path);

    static std::vector<DepthData> loadDepth(const std::string& path);

    static std::vector<GPSData> loadGPS(const std::string& path);

    static std::vector<MagData> loadMag(const std::string& path);

    static void saveResults(
        const std::vector<NavState>& results,
        const std::string& path
    );

    static void generateSyntheticData(
        const std::string& imu_out,
        const std::string& dvl_out,
        const std::string& depth_out,
        const std::string& gps_out,
        const std::string& mag_out,
        double duration = 80.0,
        double dt_imu   = 0.01,
        double dt_dvl   = 0.1,
        double dt_depth = 0.5,
        double dt_gps   = 1.0,
        double dt_mag   = 0.2
    );

private:
    static std::string trim(const std::string& s);

    static std::vector<std::string> split(
        const std::string& line,
        char delim = ','
    );
};

} 