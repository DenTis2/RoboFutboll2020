#include "SLAM.hpp"
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>

using namespace std;
SLAM::SLAM(LidarReader& lidar_, Map& map_, MotorController& motor_, const RobotConfig& cfg)
: lidar(lidar_), map(map_), motor(motor_), config(cfg), pose(0.0,0.0,0.0) { }

Eigen::Vector2d SLAM::polarToCartesian(double angle, double distance) const {
    return Eigen::Vector2d(distance * std::cos(angle), distance * std::sin(angle));
}

double SLAM::median_of_vector(std::vector<double>& v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    return (n % 2 == 1) ? v[n/2] : 0.5 * (v[n/2 - 1] + v[n/2]);
}

/*
  scanMatch:
    - сопоставляет новые лучи и предыдущие по ключу = округлённый angle*ANGLE_SCALE
    - сначала собирает кандидаты dtheta (локальные угловые отличия через atan2)
    - берёт медиану dtheta
    - поворачивает newScan на эту медиану и берёт медианы dx/dy
    - возвращает true если статистика достаточна
*/
bool SLAM::scanMatch(const std::vector<LidarPoint>& newScan,
                     double &out_tx, double &out_ty, double &out_dtheta)
{
    out_tx = out_ty = out_dtheta = 0.0;

    if (newScan.empty()) return false;
    if (lastScan.empty()) {
        // первый скан — просто запомним
        lastScan = newScan;
        return false;
    }

    const double ANGLE_SCALE = 1000.0; // масштаб для квантования угла
    std::unordered_map<int, Eigen::Vector2d> prevMap;
    prevMap.reserve(lastScan.size()*2);

    // build prev map: key -> cartesian point
    for (const auto &p : lastScan) {
        if (!std::isfinite(p.angle) || !std::isfinite(p.distance)) continue;
        if (p.distance <= 0.01 || p.distance > 50.0) continue;
        int key = static_cast<int>(std::round(p.angle * ANGLE_SCALE));
        prevMap[key] = polarToCartesian(p.angle, p.distance);
    }

    // collect candidate dtheta from matched angles
    std::vector<double> thetas;
    thetas.reserve(512);

    for (const auto &p : newScan) {
        if (!std::isfinite(p.angle) || !std::isfinite(p.distance)) continue;
        if (p.distance <= 0.01 || p.distance > 50.0) continue;
        int key = static_cast<int>(std::round(p.angle * ANGLE_SCALE));
        auto it = prevMap.find(key);
        if (it == prevMap.end()) continue;

        Eigen::Vector2d prevPt = it->second;
        Eigen::Vector2d newPt = polarToCartesian(p.angle, p.distance);

        double a1 = std::atan2(prevPt.y(), prevPt.x());
        double a2 = std::atan2(newPt.y(), newPt.x());
        double dth = a1 - a2;
        // normalize
        while (dth > M_PI) dth -= 2.0*M_PI;
        while (dth < -M_PI) dth += 2.0*M_PI;
        thetas.push_back(dth);
    }

    // need enough matches
    const size_t MIN_THETA_MATCHES = 20;
    cout<<"THETAS.SIZE()!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<thetas.size()<<endl;
    if (thetas.size() < MIN_THETA_MATCHES) {
        // not enough reliable angle matches -> update lastScan and bail out
        lastScan = newScan;
        return false;
    }

    // median dtheta
    out_dtheta = median_of_vector(*(new std::vector<double>(thetas))); // copy for median func
    // (the median_of_vector sorts the vector in-place; using a temp avoids re-sorting below)

    // Now compute dx/dy after rotating new points by med_dtheta
    std::vector<double> dxs, dys;
    dxs.reserve(512); dys.reserve(512);

    for (const auto &p : newScan) {
        if (!std::isfinite(p.angle) || !std::isfinite(p.distance)) continue;
        if (p.distance <= 0.01 || p.distance > 50.0) continue;
        int key = static_cast<int>(std::round(p.angle * ANGLE_SCALE));
        auto it = prevMap.find(key);
        if (it == prevMap.end()) continue;

        Eigen::Vector2d prevPt = it->second;
        Eigen::Vector2d newPt = polarToCartesian(p.angle, p.distance);

        // rotate newPt by out_dtheta
        double c = std::cos(out_dtheta), s = std::sin(out_dtheta);
        double xr = newPt.x() * c - newPt.y() * s;
        double yr = newPt.x() * s + newPt.y() * c;

        dxs.push_back(prevPt.x() - xr);
        dys.push_back(prevPt.y() - yr);
    }

    const size_t MIN_XY_MATCHES = 20;
    cout<<"DXS.SIZE()<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<dxs.size()<<" DYS.SIZE()>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<dys.size()<<endl;
    if (dxs.size() < MIN_XY_MATCHES || dys.size() < MIN_XY_MATCHES) {
        lastScan = newScan;
        return false;
    }
    
    // median dx/dy
    out_tx = median_of_vector(*(new std::vector<double>(dxs)));
    out_ty = median_of_vector(*(new std::vector<double>(dys)));

    // Free temporaries created above via new
    // (we used new to avoid modifying original vectors passed to median_of_vector)
    // Note: better approach would avoid allocations, but it's fine and explicit.

    // update lastScan only after successful match
    lastScan = newScan;
    return true;
}

void SLAM::step() {
    std::vector<LidarPoint> scan;
    if (!lidar.readScan(scan) || scan.empty()) {
        return;
    }

    // protection: require minimal number of rays
    if (scan.size() < 8) {
        lastScan = scan;
        return;
    }

    std::cout << "[DEBUG] Scan points received: " << scan.size() << std::endl;

    // try to compute relative transform between lastScan and scan
    double tx=0.0, ty=0.0, dtheta=0.0;
    bool matched = scanMatch(scan, tx, ty, dtheta);

    if (matched) {
        // apply transform: tx/ty are in robot-local coordinates (relative to previous scan orientation)
        // We must rotate tx/ty into global coordinates using current pose yaw BEFORE adding yaw change,
        // OR apply yaw increment first then rotate tx/ty by old yaw — both are equivalent if carefully handled.
        // We'll rotate by current pose yaw (pose(2)), then update yaw.
        double cosYaw = std::cos(pose(2));
        double sinYaw = std::sin(pose(2));
        double global_dx = tx * cosYaw - ty * sinYaw;
        double global_dy = tx * sinYaw + ty * cosYaw;
        cout<<"GLOABAL_DX: "<<global_dx<<" DY: "<<global_dy<<" DTHETA: "<<dtheta<<endl;
        pose(0) += global_dx;
        pose(1) += global_dy;
        pose(2) += dtheta;

        // normalize yaw
        while (pose(2) > M_PI) pose(2) -= 2.0*M_PI;
        while (pose(2) < -M_PI) pose(2) += 2.0*M_PI;
    } else {
        // if not matched, pose stays as is; lastScan already updated inside scanMatch on failure
    }

    Eigen::Vector3d currentPose = getPose();
    std::cout << "Pose: " << currentPose.transpose() << std::endl;

    // prepare points for map update (convert to robot-local cartesian)
    std::vector<Eigen::Vector2d> points;
    points.reserve(scan.size());
    for (const auto &p : scan) {
        if (!std::isfinite(p.angle) || !std::isfinite(p.distance)) continue;
        if (p.distance <= 0.01 || p.distance > 50.0) continue;
        points.push_back(polarToCartesian(p.angle, p.distance));
    }

    std::cout << "[DEBUG] Points to map: " << points.size() << std::endl;
    if (!points.empty()) {
         map.update(points, currentPose);
    }

    // debug: quick occupancy count (you already have similar code)
    int occupied = 0;
    for (int x = 0; x < map.width; ++x)
       for (int y = 0; y < map.height; ++y)
           if (!map.isFree(x, y)) ++occupied;
    std::cout << "[DEBUG] Occupied cells in map: " << occupied << std::endl;
    map.debugPrintMap();
}

Eigen::Vector3d SLAM::getPose() const { return pose; }
