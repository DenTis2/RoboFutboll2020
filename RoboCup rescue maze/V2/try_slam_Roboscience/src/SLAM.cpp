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
/*
// --- scanMatch left as you had (not repeated here) ---
// (I assume you keep your scanMatch implementation, unchanged)
bool SLAM::scanMatch(const std::vector<LidarPoint>& newScan,
                     double &out_tx, double &out_ty, double &out_dtheta)
{
    cout<<"SCAN_MATCHP_ALIVE$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<endl;
    // Must have a baseline scan
    if (lastScan.empty()) {
        // Cannot match yet — accept this scan as baseline
        lastScan = newScan;
        out_tx = out_ty = out_dtheta = 0.0;
        cout<<"NNNNOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO_IM_NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO_ALIVE!"<<endl;
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
    cout<<"DMX: "<<mdx<<"DMY: "<<mdy<<"DMTH: "<<mdth<<"_______________________________&&&&&&&&&&&&&&&&&&&&&&&&^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;
    // Clamp rotation (avoid crazy sips)
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
}*/
bool SLAM::scanMatch(const std::vector<LidarPoint>& newScan,
                     double &out_tx, double &out_ty, double &out_dtheta)
{
    out_tx = out_ty = out_dtheta = 0.0;

    if (newScan.empty()) return false;
    if (lastScan.empty()) {
        lastScan = newScan;
        return false;
    }

    // Build a quick lookup of previous scan by rounded degree [0..359]
    const double RAD2DEG = 180.0 / M_PI;
    std::array<std::vector<Eigen::Vector2d>, 360> prevBuckets;
    for (const auto &p : lastScan) {
        if (!std::isfinite(p.angle) || !std::isfinite(p.distance)) continue;
        if (p.distance <= 0.01 || p.distance > 50.0) continue;
        int deg = static_cast<int>(std::round(p.angle * RAD2DEG)) % 360;
        if (deg < 0) deg += 360;
        prevBuckets[deg].push_back(polarToCartesian(p.angle, p.distance));
    }

    // Collect matched pairs (newPt, prevPt)
    std::vector<Eigen::Vector2d> A; // new points
    std::vector<Eigen::Vector2d> B; // prev points (matching)
    A.reserve(newScan.size());
    B.reserve(newScan.size());

    for (const auto &p : newScan) {
        if (!std::isfinite(p.angle) || !std::isfinite(p.distance)) continue;
        if (p.distance <= 0.01 || p.distance > 50.0) continue;

        int deg = static_cast<int>(std::round(p.angle * RAD2DEG)) % 360;
        if (deg < 0) deg += 360;

        // try same degree and neighbors +/-1 deg for robustness
        bool matched = false;
        for (int off = -1; off <= 1 && !matched; ++off) {
            int k = (deg + off + 360) % 360;
            if (prevBuckets[k].empty()) continue;
            // choose median-of-bucket (robust) or last — choose first here (bucket is usually one)
            const Eigen::Vector2d &prevPt = prevBuckets[k].front();
            A.push_back(polarToCartesian(p.angle, p.distance));
            B.push_back(prevPt);
            matched = true;
        }
    }

    const size_t MIN_MATCHES = 30;
    if (A.size() < MIN_MATCHES || B.size() < MIN_MATCHES) {
        // not enough matches -> keep lastScan as-is
        out_tx = out_ty = out_dtheta = 0.0;
        return false;
    }

    // Compute best-fit rotation (2D) from A -> B using closed form
    // theta = atan2( sum(nx*py - ny*px), sum(nx*px + ny*py) )
    double S_num = 0.0;
    double S_den = 0.0;
    for (size_t i = 0; i < A.size(); ++i) {
        double nx = A[i].x(), ny = A[i].y();
        double px = B[i].x(), py = B[i].y();
        S_num += (nx * py - ny * px);
        S_den += (nx * px + ny * py);
    }

    double dtheta = std::atan2(S_num, S_den);

    // normalize angle
    while (dtheta > M_PI) dtheta -= 2.0*M_PI;
    while (dtheta < -M_PI) dtheta += 2.0*M_PI;

    // Reject insane rotations (flip-by-pi etc.)
    const double MAX_ACCEPTED_ROT = 0.8; // radians, tuneable (0.8 ~ 45deg)
    if (std::fabs(dtheta) > MAX_ACCEPTED_ROT) {
        out_tx = out_ty = out_dtheta = 0.0;
        return false;
    }

    // rotate new points by dtheta and collect translations (prev - rotated_new)
    double c = std::cos(dtheta), s = std::sin(dtheta);
    std::vector<double> dxs, dys;
    dxs.reserve(A.size()); dys.reserve(A.size());
    for (size_t i = 0; i < A.size(); ++i) {
        double nx = A[i].x(), ny = A[i].y();
        double rx = nx * c - ny * s;
        double ry = nx * s + ny * c;
        double px = B[i].x(), py = B[i].y();
        dxs.push_back(px - rx);
        dys.push_back(py - ry);
    }

    // take medians (robust)
    auto median = [](std::vector<double> v)->double {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        size_t n = v.size();
        return (n % 2) ? v[n/2] : 0.5 * (v[n/2 - 1] + v[n/2]);
    };

    double tx = median(dxs);
    double ty = median(dys);

    // sanity checks on tx/ty magnitudes (avoid huge jumps)
    const double MAX_TRANSLATION = 5.0; // meters
    if (!std::isfinite(tx) || !std::isfinite(ty) ||
        std::fabs(tx) > MAX_TRANSLATION || std::fabs(ty) > MAX_TRANSLATION) {
        out_tx = out_ty = out_dtheta = 0.0;
        return false;
    }

    // success -> output
    out_tx = tx;
    out_ty = ty;
    out_dtheta = dtheta;

    // update lastScan only when match succeeded
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
//            integrateOdometry(dt);
            // do not copy odom to pose until we have some correction policy
//            pose = odomPose;
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
//    integrateOdometry(dt);

    // 2) convert scan to cartesian
    std::cout << "[DEBUG] Scan points received: " << scan.size() << std::endl;

    std::vector<Eigen::Vector2d> points;
    points.reserve(scan.size());
    for (const auto &p : scan) {
        if (!std::isfinite(p.angle) || !std::isfinite(p.distance)) continue;
        if (p.distance <= 0.01 || p.distance > 50.0) continue;
        points.push_back(polarToCartesian(p.angle, p.distance));
	cout<<"_____________________MAIN_ERROR: dist: "<<p.distance<<" angle: "<<p.angle<<" cord: "<<polarToCartesian(p.angle,p.distance)<<endl;
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
