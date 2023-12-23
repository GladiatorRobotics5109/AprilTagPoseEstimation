cmake_minimum_required(VERSION 3.0.0)
project(AprilTagPoseEstimation VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

set(AprilTagLib /usr/local/lib/libapriltag.a)

find_package(OpenCV REQUIRED)

find_package(apriltag REQUIRED)

file(GLOB AprilTagPoseEstimation_SRC
    "src/**.cpp"
    "src/common/*.cpp"
)

add_executable(AprilTagPoseEstimation ${AprilTagPoseEstimation_SRC})

target_include_directories(AprilTagPoseEstimation PUBLIC /usr/local/include)

target_link_libraries(AprilTagPoseEstimation ${OpenCV_LIBS} ${AprilTagLib})
