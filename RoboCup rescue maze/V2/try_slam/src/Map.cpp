// Map.cpp — обновлённый
#include "Map.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

Map::Map(int w, int h, double res) : width(w), height(h), resolution(res) {
    // раньше было 0 — это было FREE по умолчанию. Делать так неправильно.
    grid.resize(width, std::vector<int>(height, Map::UNKNOWN));
}

// Bresenham integer line between (x0,y0) and (x1,y1)
static void bresenham_line(int x0, int y0, int x1, int y1, std::vector<std::pair<int,int>>& out) {
    out.clear();
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    while (true) {
        out.emplace_back(x, y);
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
}

void Map::update(const std::vector<Eigen::Vector2d>& points, const Eigen::Vector3d& robotPose) {
    int updatedPoints = 0;

    // robot grid coords
    auto [rx, ry] = worldToGrid(robotPose(0), robotPose(1));

    std::vector<std::pair<int,int>> ray;
    ray.reserve(256);

    for (auto& pt : points) {
        // transform point (robot-local) into world
        double x_world = robotPose(0) + pt(0) * cos(robotPose(2)) - pt(1) * sin(robotPose(2));
        double y_world = robotPose(1) + pt(0) * sin(robotPose(2)) + pt(1) * cos(robotPose(2));

        if (!std::isfinite(x_world) || !std::isfinite(y_world)) continue;
        if (std::fabs(x_world) > width * resolution || std::fabs(y_world) > height * resolution) continue;

        int xi = int((x_world + width * resolution / 2) / resolution);
        int yi = int((y_world + height * resolution / 2) / resolution);

        if (xi < 0 || yi < 0 || xi >= width || yi >= height) continue;

        // Raycast from robot grid cell to (xi, yi)
        bresenham_line(rx, ry, xi, yi, ray);

        // mark intermediate cells as FREE, endpoint as OCCUPIED
        // ignore the last cell for "freeing" so endpoint becomes occupied
        for (size_t k = 0; k + 1 < ray.size(); ++k) {
            int cx = ray[k].first, cy = ray[k].second;
            if (cx >= 0 && cy >= 0 && cx < width && cy < height) {
                // only change UNKNOWN -> FREE; do not overwrite OCCUPIED
                if (grid[cx][cy] == Map::UNKNOWN||grid[cx][cy] == Map::OCCUPIED) grid[cx][cy] = Map::FREE;
            }
        }
        // endpoint
        int ex = ray.back().first, ey = ray.back().second;
        if (ex >= 0 && ey >= 0 && ex < width && ey < height) {
            grid[ex][ey] = Map::OCCUPIED;
            updatedPoints++;
        }
    }

    std::cout << "[DEBUG] Updated points in map: " << updatedPoints << std::endl;
}

void Map::inflateObstacles(double robotWidth, double robotLength) {
    int rx = int(robotWidth / resolution);
    int ry = int(robotLength / resolution);

    std::vector<std::vector<int>> newGrid = grid;
    for (int x = 0; x < width; ++x)
        for (int y = 0; y < height; ++y)
            if (grid[x][y] == Map::OCCUPIED)
                for (int dx=-rx; dx<=rx; ++dx)
                    for (int dy=-ry; dy<=ry;++dy){
                        int nx = x+dx, ny=y+dy;
                        if(nx>=0 && nx<width && ny>=0 && ny<height)
                            newGrid[nx][ny] = Map::OCCUPIED;
                    }
    grid = std::move(newGrid);
}

bool Map::isFree(int x, int y) const {
    if(x<0 || x>=width || y<0 || y>=height) return false;
    return grid[x][y] == Map::FREE;
}

Eigen::Vector2d Map::gridToWorld(int gx, int gy) const {
    return Eigen::Vector2d((gx+0.5)*resolution - width*resolution/2.0,
                           (gy+0.5)*resolution - height*resolution/2.0);
}

std::pair<int,int> Map::worldToGrid(double x, double y) const {
    return {int((x + width*resolution/2)/resolution), int((y + height*resolution/2)/resolution)};
}

void Map::debugPrintMap(const Eigen::Vector3d& robotPose) const {
    std::cout << "Map visualization:\n";

    // print a compact ASCII map, . = free, # = occ, ' ' = unknown
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            int cell = grid[x][y];
            if (cell == Map::OCCUPIED) std::cout << "#";
            else if (cell == Map::FREE) std::cout << ".";
            else std::cout << " ";
        }
        std::cout << "\n";
    }

    auto [rx, ry] = worldToGrid(robotPose(0), robotPose(1));
    std::cout << "Map size: width="<<width<<", height="<<height<<", resolution="<<resolution<<"\n";
    std::cout << "Robot grid pos: ("<<rx<<","<<ry<<")  pose_world: "<<robotPose.transpose()<<"\n";
}
