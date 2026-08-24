// Map.hpp — обновлённый
#pragma once
#include <vector>
#include <Eigen/Dense>

class Map {
public:
    Map(int width, int height, double resolution);
    void update(const std::vector<Eigen::Vector2d>& points, const Eigen::Vector3d& robotPose);
    void inflateObstacles(double robotWidth, double robotLength);
    void debugPrintMap(const Eigen::Vector3d& robotPose) const;
    bool isFree(int x, int y) const;
    Eigen::Vector2d gridToWorld(int gx, int gy) const;
    std::pair<int,int> worldToGrid(double x, double y) const;

    int getCell(int x, int y) const {
       if (x < 0 || x >= width || y < 0 || y >= height) return -1; // за границами карты
       return grid[x][y];
     }

    int width, height;
    double resolution;

    // occupancy values
    static constexpr int UNKNOWN = -1;
    static constexpr int FREE    = 0;
    static constexpr int OCCUPIED= 1;

private:
    std::vector<std::vector<int>> grid;
};
