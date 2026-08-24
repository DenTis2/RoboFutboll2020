#include "Explorer.hpp"
#include <queue>
#include <cmath>
#include <algorithm>
using namespace std;
Explorer::Explorer(Map& map_, SLAM& slam_, MotorController& motor_, const RobotConfig& cfg)
: map(map_), slam(slam_), motor(motor_), config(cfg) {}

void Explorer::step() {
    Eigen::Vector2d robotPos = slam.getPose().head<2>();

    cout<<"_____________________________________________________________________________________________________________"<<endl;
    // Если путь пуст, ищем новый фронтир
    if(currentPath.empty()){
        Eigen::Vector2d target = findNextFrontier();
        std::cout << "[DEBUG] Robot: " << robotPos(0) << " " << robotPos(1) 
                  << " Next frontier: " << target(0) << " " << target(1) << std::endl;

        if((target - robotPos).norm() < 1e-6) {
            std::cout << "[DEBUG] No new frontier found, stopping" << std::endl;
            currentPath.clear();
            motor.setSpeed(0.0, 0.0);
            cout<<"T: "<<fixed<<(target-robotPos).norm()<<" OAOAOAOAOAOOAOAOAOAOAOAOAOOAOAOAOAOAOAOAOOAAOOAOAOAOAOOAOOAOAOOAOAOAOAO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
            return;
        }

        currentPath = planPath(robotPos, target);
        std::cout << "[DEBUG] Path size: " << currentPath.size() << std::endl;
    }

    // Двигаемся по пути, если он не пуст
    moveAlongPath();
}

/*void Explorer::step() {
    Eigen::Vector2d robotPos = slam.getPose().head<2>();
    if(!std::isfinite(robotPos(0)) || !std::isfinite(robotPos(1))) {
        motor.setSpeed(0.0, 0.0);
        return;
    }

    if(currentPath.empty()) {
        Eigen::Vector2d target = findNextFrontier();

        // защита: если фронтир совпадает с текущей позой
        if((target - robotPos).norm() < 1e-6) {
            currentPath.clear();
            motor.setSpeed(0.0, 0.0);
            return;
        }

        currentPath = planPath(robotPos, target);

        // если путь не найден
        if(currentPath.empty()) {
            motor.setSpeed(0.0, 0.0);
            return;
        }

        // защита от пути длиной 1, где точка == текущей позиции
        if(currentPath.size() == 1 && (currentPath.front() - robotPos).norm() < 1e-6) {
            currentPath.clear();
            motor.setSpeed(0.0, 0.0);
            return;
        }
    }

    moveAlongPath();
}/*
void Explorer::step() {
    Eigen::Vector2d robotPos = slam.getPose().head<2>();

    if(currentPath.empty()){
        Eigen::Vector2d target = findNextFrontier();

        // --- Debug: показать фронтир и позицию робота ---
        std::cout << "[DEBUG] Robot: " << robotPos.transpose() 
                  << " Next frontier: " << target.transpose() << std::endl;

        if((target - robotPos).norm() < 1e-6) {
            currentPath.clear();
            std::cout << "[DEBUG] No new frontier found, stopping" << std::endl;
            return;
        }

        currentPath = planPath(robotPos, target);

        // --- Debug: показать размер пути и первые точки ---
        std::cout << "[DEBUG] Planned path size: " << currentPath.size() << std::endl;
        if(!currentPath.empty())
            std::cout << "[DEBUG] First waypoint: " << currentPath.front().transpose() << std::endl;

        return; // не двигаем робот пока не проверили путь
    }

    // --- Debug: показать движение к текущему waypoint ---
    Eigen::Vector2d target = currentPath.front();
    std::cout << "[DEBUG] Moving towards: " << target.transpose() << std::endl;

    // можно тут пока не шлём setSpeed, чтобы робот не ехал
}
*/


/*void Explorer::step() {
    if(currentPath.empty()){
        Eigen::Vector2d target = findNextFrontier();
        currentPath = planPath(slam.getPose().head<2>(), target);
        if(currentPath.empty()) return;
    }
    moveAlongPath();
}*/

Eigen::Vector2d Explorer::findNextFrontier() {
    Eigen::Vector2d robotPos = slam.getPose().head<2>();
    auto [rx, ry] = map.worldToGrid(robotPos(0), robotPos(1));

    double bestDist = 1e9;
    Eigen::Vector2d bestFrontier = robotPos;

    for (int x = 1; x < map.width - 1; ++x) {
        for (int y = 1; y < map.height - 1; ++y) {
            if (map.getCell(x, y) == 0) {  // свободная клетка
                bool frontier = false;

                for (int dx = -1; dx <= 1 && !frontier; ++dx) {
                    for (int dy = -1; dy <= 1 && !frontier; ++dy) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (map.getCell(nx, ny) == -1)
                            frontier = true;
                    }
                }

                if (frontier) {
                    double dist = std::hypot(x - rx, y - ry);
                    if (dist < bestDist && dist > 0.5) {//3.0
                        bestDist = dist;
                        bestFrontier = Eigen::Vector2d(x, y);
                    }
                }
            }
        }
    }

    if (bestDist < 1e9) {
        Eigen::Vector2d world = map.gridToWorld((int)bestFrontier.x(), (int)bestFrontier.y());
        std::cout << "[DEBUG] Next frontier (world): " << world.transpose() << std::endl;
        return world;
    } else {
        std::cout << "[DEBUG] No frontier found, staying in place" << std::endl;
        return robotPos;
    }
}


/*Eigen::Vector2d Explorer::findNextFrontier() {
    auto pose = slam.getPose().head<2>();
    auto [gx, gy] = map.worldToGrid(pose(0), pose(1));

    gx = std::clamp(gx, 0, map.width-1);
    gy = std::clamp(gy, 0, map.height-1);

    std::queue<std::pair<int,int>> q;
    std::vector<std::vector<bool>> visited(map.width, std::vector<bool>(map.height,false));
    q.push({gx,gy});
    visited[gx][gy]=true;

    while(!q.empty()){
        auto [x,y]=q.front(); q.pop();
        for(int dx=-1;dx<=1;++dx)
            for(int dy=-1;dy<=1;++dy){
                int nx=x+dx, ny=y+dy;
                if(nx<0||ny<0||nx>=map.width||ny>=map.height) continue;
                if(visited[nx][ny]) continue;
                visited[nx][ny]=true;
                if(map.isFree(nx,ny)) q.push({nx,ny});
                else return map.gridToWorld(nx,ny);
            }
    }
    return slam.getPose().head<2>();
}

*/
std::vector<Eigen::Vector2d> Explorer::planPath(const Eigen::Vector2d& start, const Eigen::Vector2d& goal) {

    auto [sx, sy] = map.worldToGrid(start(0), start(1));
    auto [gx, gy] = map.worldToGrid(goal(0), goal(1));

    sx = std::clamp(sx, 0, map.width-1);
    sy = std::clamp(sy, 0, map.height-1);
    gx = std::clamp(gx, 0, map.width-1);
    gy = std::clamp(gy, 0, map.height-1);

    struct Node {
        int x, y;
        double g, h;
        int parentIdx; // индекс родителя в nodes, -1 если нет
    };

    std::vector<Node> nodes;
    std::vector<std::vector<int>> closed(map.width, std::vector<int>(map.height, -1)); // хранит индекс Node в nodes

    auto cmp = [&](int aIdx, int bIdx) {
        const Node &a = nodes[aIdx], &b = nodes[bIdx];
        return (a.g + a.h) > (b.g + b.h);
    };

    std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);

    nodes.push_back({sx, sy, 0.0, std::hypot(gx - sx, gy - sy), -1});
    open.push(0);

    int goalIdx = -1;

    while(!open.empty()) {
        int idx = open.top(); open.pop();
        Node &n = nodes[idx];

        if(closed[n.x][n.y] != -1) continue;
        closed[n.x][n.y] = idx;

        if(n.x == gx && n.y == gy) {
            goalIdx = idx;
            break;
        }

        for(int dx = -1; dx <= 1; ++dx) {
            for(int dy = -1; dy <= 1; ++dy) {
                if(dx == 0 && dy == 0) continue;
                int nx = n.x + dx, ny = n.y + dy;
                if(nx < 0 || ny < 0 || nx >= map.width || ny >= map.height) continue;
                if(!map.isFree(nx, ny) || closed[nx][ny] != -1) continue;

                nodes.push_back({nx, ny, n.g + std::hypot(dx, dy), std::hypot(gx - nx, gy - ny), idx});
                open.push(nodes.size() - 1);
            }
        }
    }

    // Если путь не найден
    if(goalIdx == -1) return {};

    // Восстановление пути
    std::vector<Eigen::Vector2d> path;
    int idx = goalIdx;
    while(idx != -1) {
        Node &n = nodes[idx];
        path.push_back(map.gridToWorld(n.x, n.y));
        idx = n.parentIdx;
    }
    std::reverse(path.begin(), path.end());
    std::cout << "Start: " << sx << "," << sy << " Goal: " << gx << "," << gy << " Path size: " << path.size() << "\n";
    return path;
}
void Explorer::moveAlongPath() {
    if(currentPath.empty()){
	cout<<"currentPATH.epmty()!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
	return;
    }
    cout<<"I IN MOVEALONGPATH!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
    Eigen::Vector2d pos = slam.getPose().head<2>();
    double robotYaw = slam.getPose()(2);

    if(!std::isfinite(pos(0)) || !std::isfinite(pos(1)) || !std::isfinite(robotYaw)) return;

    Eigen::Vector2d target = currentPath.front();
    Eigen::Vector2d dir = target - pos;
    double distance = dir.norm();

/*    if(distance < 0.01) {
        currentPath.erase(currentPath.begin());
        motor.setSpeed(0.0, 0.0);
        return;
    }*/

    double angle_to_target = atan2(dir(1), dir(0));
    double angle_error = angle_to_target - robotYaw;

    // нормализация угла к [-pi, pi]
    while(angle_error > M_PI) angle_error -= 2*M_PI;
    while(angle_error < -M_PI) angle_error += 2*M_PI;

    double left = 0.0, right = 0.0;

    if(std::abs(angle_error) > config.tankTurnThreshold) {
        // танковый разворот
        left  = (angle_error > 0) ? -config.maxSpeed/2 : config.maxSpeed/2;
        right = -left;
    } else {
        // локальный PID прямо в функции
        static double prev_error = 0.0;
        static double integral = 0.0;

        integral += angle_error;
        double derivative = angle_error - prev_error;
        prev_error = angle_error;

        double correction = config.pid_kp*angle_error + config.pid_ki*integral + config.pid_kd*derivative;
        double baseSpeed = config.maxSpeed/2;
        std::cout<<"CORRECTION:"<<correction<<std::endl;
        left  = std::clamp(baseSpeed - correction, -config.maxSpeed, config.maxSpeed);
        right = std::clamp(baseSpeed + correction, -config.maxSpeed, config.maxSpeed);
    }
    std::cout << "[MOVE] angle_error=" << angle_error; 
    std::cout<< " dist=" << distance; 
    std::cout<< " left=" << left << " right=" << right << std::endl;
    std::cout<< " pose=" << pos.transpose();
    std::cout<< " target=" << target.transpose() << std::endl;
    motor.setSpeed(left, right);
}

/*
void Explorer::moveAlongPath() {
    if(currentPath.empty()) return;

    Eigen::Vector2d pos = slam.getPose().head<2>();
    double robotYaw = slam.getPose()(2);

    // проверка NaN/Inf
    if(!std::isfinite(pos(0)) || !std::isfinite(pos(1)) || !std::isfinite(robotYaw)) {
        motor.setSpeed(0.0, 0.0);
        return;
    }

    Eigen::Vector2d target = currentPath.front();
    Eigen::Vector2d dir = target - pos;
    double distance = dir.norm();

    // если мы на цели — удаляем waypoint
    if(distance < 0.05) {
        currentPath.erase(currentPath.begin());
        motor.setSpeed(0.0, 0.0);
        return;
    }

    double angle_to_target = std::atan2(dir(1), dir(0));
    double angle_error = angle_to_target - robotYaw;

    // нормализация [-pi, pi]
    while(angle_error > M_PI) angle_error -= 2*M_PI;
    while(angle_error < -M_PI) angle_error += 2*M_PI;

    double left = 0.0, right = 0.0;

    if(std::abs(angle_error) > config.tankTurnThreshold) {
        // танковый разворот
        left  = (angle_error > 0) ? -config.maxSpeed/2 : config.maxSpeed/2;
        right = -left;
    } else {
        // PID-контроль курса
        integral += angle_error;
        double derivative = angle_error - last_error;
        last_error = angle_error;

        double correction = config.pid_kp*angle_error + config.pid_ki*integral + config.pid_kd*derivative;
        double baseSpeed = config.maxSpeed/2;
        left  = std::clamp(baseSpeed - correction, -config.maxSpeed, config.maxSpeed);
        right = std::clamp(baseSpeed + correction, -config.maxSpeed, config.maxSpeed);
    }

    motor.setSpeed(left, right);
}
*/

/*void Explorer::moveAlongPath() {
    if(currentPath.empty()) return;

    Eigen::Vector2d pos = slam.getPose().head<2>();
    double robotYaw = slam.getPose()(2);

    if(!std::isfinite(pos(0)) || !std::isfinite(pos(1)) || !std::isfinite(robotYaw)) return;

    Eigen::Vector2d target = currentPath.front();
    Eigen::Vector2d dir = target - pos;
    double distance = dir.norm();

    if(distance < 0.05) {
        currentPath.erase(currentPath.begin());
        return;
    }

    double angle_to_target = atan2(dir(1), dir(0));
    double angle_error = angle_to_target - robotYaw;

    while(angle_error > M_PI) angle_error -= 2*M_PI;
    while(angle_error < -M_PI) angle_error += 2*M_PI;

    double left = 0.0, right = 0.0;

    if(std::abs(angle_error) > config.tankTurnThreshold) {
        left  = (angle_error > 0) ? -config.maxSpeed/2 : config.maxSpeed/2;
        right = -left;
    } else {
        integral += angle_error;
        double derivative = angle_error - last_error;
        last_error = angle_error;
        double correction = config.pid_kp*angle_error + config.pid_ki*integral + config.pid_kd*derivative;
        double baseSpeed = config.maxSpeed/2;
        left  = std::clamp(baseSpeed - correction, -config.maxSpeed, config.maxSpeed);
        right = std::clamp(baseSpeed + correction, -config.maxSpeed, config.maxSpeed);
    }

    motor.setSpeed(left, right);
}*/


/*void Explorer::moveAlongPath() {
    if(currentPath.empty()) return;

    Eigen::Vector2d pos = slam.getPose().head<2>();
    double robotYaw = slam.getPose()(2);

    // Берем первый waypoint
    Eigen::Vector2d target = currentPath.front();
    Eigen::Vector2d dir = target - pos;
    double distance = dir.norm();

    // Если мы почти на цели — удаляем waypoint
    if(distance < 0.05) {
        currentPath.erase(currentPath.begin());
        return;
    }

    double angle_to_target = std::atan2(dir(1), dir(0));
    double angle_error = angle_to_target - robotYaw;

    // Нормализация угла к [-pi, pi]
    while(angle_error > M_PI) angle_error -= 2*M_PI;
    while(angle_error < -M_PI) angle_error += 2*M_PI;

    double left = 0.0, right = 0.0;

    if(std::abs(angle_error) > config.tankTurnThreshold) {
        // Танковый разворот
        left  = (angle_error > 0) ? -config.maxSpeed/2 : config.maxSpeed/2;
        right = -left;
    } else {
        // PID-контроль курса
        integral += angle_error;
        double derivative = angle_error - last_error;
        last_error = angle_error;

        double correction = config.pid_kp*angle_error + config.pid_ki*integral + config.pid_kd*derivative;

        double baseSpeed = config.maxSpeed/2;
        left  = std::clamp(baseSpeed - correction, -config.maxSpeed, config.maxSpeed);
        right = std::clamp(baseSpeed + correction, -config.maxSpeed, config.maxSpeed);
    }

    // Отправка команд на моторы
    motor.setSpeed(left, right);
}*/

/*
std::vector<Eigen::Vector2d> Explorer::planPath(const Eigen::Vector2d& start, const Eigen::Vector2d& goal){
    auto [sx,sy]=map.worldToGrid(start(0),start(1));
    auto [gx,gy]=map.worldToGrid(goal(0),goal(1));

    struct Node { int x,y; double g,h; Node* parent; };
    auto cmp = [](Node* a, Node* b){ return (a->g + a->h) > (b->g + b->h); };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);
    std::vector<std::vector<bool>> closed(map.width, std::vector<bool>(map.height,false));

    Node* startNode = new Node{sx,sy,0,std::hypot(gx-sx,gy-sy),nullptr};
    open.push(startNode);

    Node* goalNode=nullptr;
    while(!open.empty()){
        Node* n=open.top(); open.pop();
        if(closed[n->x][n->y]) { delete n; continue; }
        closed[n->x][n->y]=true;
        if(n->x==gx && n->y==gy){ goalNode=n; break; }

        for(int dx=-1;dx<=1;++dx)
            for(int dy=-1;dy<=1;++dy){
                if(dx==0 && dy==0) continue;
                int nx=n->x+dx, ny=n->y+dy;
                if(nx<0||ny<0||nx>=map.width||ny>=map.height) continue;
                if(closed[nx][ny] || !map.isFree(nx,ny)) continue;
                Node* child = new Node{nx,ny,n->g+std::hypot(dx,dy),std::hypot(gx-nx,gy-ny),n};
                open.push(child);
            }
    }
    while(!open.empty()){delete open.top();open.pop();}
    if(goalNode==nullptr) return {};
    std::vector<Eigen::Vector2d> path;
    while(goalNode){
        path.push_back(map.gridToWorld(goalNode->x,goalNode->y));
        goalNode=goalNode->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void Explorer::moveAlongPath(){
    if(currentPath.empty()) return;
    Eigen::Vector2d target=currentPath.front();
    Eigen::Vector2d pos=slam.getPose().head<2>();
    Eigen::Vector2d dir=target-pos;
    double angle_to_target=atan2(dir(1),dir(0));
    double angle_error=angle_to_target-slam.getPose()(2);

    double left, right;

    if(std::abs(angle_error) > config.tankTurnThreshold){
        // Танковый разворот
        left  = (angle_error>0)? -config.maxSpeed/2 : config.maxSpeed/2;
        right = -left;
    } else {
        // PID-контроль курса
        integral += angle_error;
        double derivative = angle_error - last_error;
        last_error = angle_error;
        double correction = config.pid_kp*angle_error + config.pid_ki*integral + config.pid_kd*derivative;

        double baseSpeed = config.maxSpeed/2;
        left  = std::clamp(baseSpeed - correction, -config.maxSpeed, config.maxSpeed);
        right = std::clamp(baseSpeed + correction, -config.maxSpeed, config.maxSpeed);
    }

    motor.setSpeed(left,right);

    if(dir.norm()<0.05) currentPath.erase(currentPath.begin());
}
*/
