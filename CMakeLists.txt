cmake_minimum_required(VERSION 3.0.0)
project(AprilTagPoseEstimation VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

set(AprilTagLib /usr/local/lib/libapriltag.a)

find_package(OpenCV REQUIRED)

find_package(apriltag REQUIRED)

add_executable(AprilTagPoseEstimation main.cpp)

target_link_libraries(AprilTagPoseEstimation ${OpenCV_LIBS} ${AprilTagLib})