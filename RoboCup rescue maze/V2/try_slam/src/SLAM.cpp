// SLAM.cpp — обновлённый (только ключевые изменения и функции)
#include "SLAM.hpp"
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>

using namespace std;

SLAM::SLAM(LidarReader& lidar_, Map& map_, MotorController& motor_, const RobotConfig& cfg)
: lidar(lidar_), map(map_), motor(motor_), config(cfg), pose(0.0,0.0,0.0), odomPose(0.0,0.0,0.0) {
    lastTime = std::chrono::steady_clock::now();
}

Eigen::Vector2d SLAM::polarToCartesian(double angle, double distance) const {
    return Eigen::Vector2d(distance * std::cos(angle), distance * std::sin(angle));
}

double SLAM::median_of_vector(std::vector<double>& v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    return (n % 2 == 1) ? v[n/2] : 0.5 * (v[n/2 - 1] + v[n/2]);
}

void SLAM::integrateOdometry(double dt) {
    if (dt <= 0.0) return;

    // get wheel rates (rotations per second) from LidarReader (as you said)
    double l_rps = lidar.getleft();   // assumed to be already in rotations/s (rps)
    double r_rps = lidar.getright();

    // wheel params (we use the numbers you gave)
    constexpr double WHEEL_RADIUS = 0.07;   // meters (7 cm)
    double wheel_base = config.wheelBase;   // meters (you set ~0.015)

    // linear velocities of wheels (m/s)
    double v_left  = l_rps * 2.0 * M_PI * WHEEL_RADIUS;
    double v_right = r_rps * 2.0 * M_PI * WHEEL_RADIUS;

    // robot linear and angular velocity
    double v = 0.5 * (v_right + v_left);
    double omega = (v_right - v_left) / wheel_base;

    // integrate on odomPose
    double theta = odomPose(2);

    // simple forward Euler (sufficient for small dt)
    double dx = v * std::cos(theta) * dt;
    double dy = v * std::sin(theta) * dt;
    double dtheta = omega * dt;

    odomPose(0) += dx;
    odomPose(1) += dy;
    odomPose(2) += dtheta;

    // normalize
    while (odomPose(2) > M_PI) odomPose(2) -= 2.0*M_PI;
    while (odomPose(2) < -M_PI) odomPose(2) += 2.0*M_PI;
}

// --- scanMatch left as you had (not repeated here) ---
// (I assume you keep your scanMatch implementation, unchanged)
bool SLAM::scanMatch(const std::vector<LidarPoint>& newScan,
                     double &out_tx, double &out_ty, double &out_dtheta)
{
    // Must have a baseline scan
    if (lastScan.empty()) {
        // Cannot match yet — accept this scan as baseline
        lastScan = newScan;
        out_tx = out_ty = out_dtheta = 0.0;
        return false;
    }

    std::vector<double> dxs, dys, dths;
    dxs.reserve(newScan.size());
    dys.reserve(newScan.size());
    dths.reserve(newScan.size());

    // -------------- EXTRACT MATCHES -------------------
    // For each point we try to find near-angle match in the previous scan
    // No full ICP, only angle-index correspondence (simple scan matcher)
    for (size_t i = 0; i < newScan.size() && i < lastScan.size(); ++i)
    {
        const auto& a = newScan[i];
        const auto& b = lastScan[i];

        if (!std::isfinite(a.angle) || !std::isfinite(a.distance)) continue;
        if (!std::isfinite(b.angle) || !std::isfinite(b.distance)) continue;

        if (a.distance <= 0.05 || b.distance <= 0.05) continue;  // too close → noisy
        if (a.distance > 50.0 || b.distance > 50.0) continue;    // too far → unreliable

        // Convert both to XY in robot-local frame
        double ax = a.distance * std::cos(a.angle);
        double ay = a.distance * std::sin(a.angle);

        double bx = b.distance * std::cos(b.angle);
        double by = b.distance * std::sin(b.angle);

        // Local difference
        dxs.push_back(bx - ax);
        dys.push_back(by - ay);
        dths.push_back(b.angle - a.angle);
    }

    // -------------- ENOUGH MATCHES? -------------------
    if (dxs.size() < 30) {
        // Too few matches — do NOT update lastScan
        out_tx = out_ty = out_dtheta = 0.0;
        return false;
    }

    // -------------- MEDIANS -------------------
    double mdx = median_of_vector(dxs);
    double mdy = median_of_vector(dys);
    double mdth = median_of_vector(dths);

    // Clamp rotation (avoid crazy slips)
    if (std::fabs(mdth) > 0.5) {
        // Likely false match
        out_tx = out_ty = out_dtheta = 0.0;
        return false;
    }

    // Return refinement
    out_tx = mdx;
    out_ty = mdy;
    out_dtheta = mdth;

    // This is a successful match → update lastScan
    lastScan = newScan;

    return true;
}

void SLAM::step() {
    std::vector<LidarPoint> scan;
    if (!lidar.readScan(scan) || scan.empty()) {
        // even if no lidar, integrate odometry with dt so robot pose moves
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        if (dt > 0.0 && dt < 1.0) { // safety clamp
            integrateOdometry(dt);
            // do not copy odom to pose until we have some correction policy
            pose = odomPose;
        }
        lastTime = now;
        return;
    }

    // compute dt
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - lastTime).count();
    if (dt <= 0 || dt > 1.0) dt = 0.01; // fallback
    lastTime = now;

    // 1) Predict with odometry
    integrateOdometry(dt);

    // 2) convert scan to cartesian
    std::cout << "[DEBUG] Scan points received: " << scan.size() << std::endl;

    std::vector<Eigen::Vector2d> points;
    points.reserve(scan.size());
    for (const auto &p : scan) {
        if (!std::isfinite(p.angle) || !std::isfinite(p.distance)) continue;
        if (p.distance <= 0.01 || p.distance > 50.0) continue;
        points.push_back(polarToCartesian(p.angle, p.distance));
    }
    std::cout << "[DEBUG] Points to map: " << points.size() << std::endl;

    // 3) Try to match scans to get correction (tx,ty,dtheta)
    double tx=0.0, ty=0.0, dtheta=0.0;
    bool matched = scanMatch(scan, tx, ty, dtheta);

    if (matched) {
        // tx/ty are the transform from newScan -> prevScan in robot-local frame
        // convert to global and apply as correction to odomPose
        double cosYaw = std::cos(odomPose(2));
        double sinYaw = std::sin(odomPose(2));
        double global_dx = tx * cosYaw - ty * sinYaw;
        double global_dy = tx * sinYaw + ty * cosYaw;
        double global_dth = dtheta;

        // Apply correction: we fuse odomPose and correction.
        // Simple strategy: apply correction as additive delta with a weight.
        // If scanMatch found a stable match, we trust it partly.
        constexpr double MATCH_TRUST = 0.9; // 0..1 — tune this (0.8 means we mostly trust lidar correction)
        odomPose(0) += global_dx * MATCH_TRUST;
        odomPose(1) += global_dy * MATCH_TRUST;
        odomPose(2) += global_dth * MATCH_TRUST;

        // normalize
        while (odomPose(2) > M_PI) odomPose(2) -= 2.0*M_PI;
        while (odomPose(2) < -M_PI) odomPose(2) += 2.0*M_PI;

        // accept newScan as lastScan (since match succeeded)
        lastScan = scan;
    } else {
        // no reliable match — do not overwrite lastScan (keeps previous reference)
        // however if lastScan is empty (startup), set it
        if (lastScan.empty()) lastScan = scan;
    }

    // set pose to odomPose (fused)
    pose = odomPose;

    // update map using fused pose and cartesian points
    if (!points.empty()) {
         map.update(points, pose);
    }

    // occupancy count
    int occupied = 0;
    for (int x = 0; x < map.width; ++x)
       for (int y = 0; y < map.height; ++y)
           if (map.getCell(x,y) == Map::OCCUPIED) ++occupied;
    std::cout << "[DEBUG] Occupied cells in map: " << occupied << std::endl;

    // print pose and map
    std::cout << "Pose: " << pose.transpose() << std::endl;
    map.debugPrintMap(pose);
}

Eigen::Vector3d SLAM::getPose() const { return pose; }
