cmake_minimum_required(VERSION 3.10)
project(RPiSLAM)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
#find_package(ASIO REQUIRED)

include_directories(include /usr/include/asio)

add_executable(rpi_slam
    src/SerialBus.cpp
    src/LidarReader.cpp
    src/MotorController.cpp
    src/Map.cpp
    src/SLAM.cpp
    src/Explorer.cpp
    src/mySerial.cpp
    main.cpp
)

target_link_libraries(rpi_slam Eigen3::Eigen pthread)
