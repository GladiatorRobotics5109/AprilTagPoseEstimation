#pragma once

#include <iostream>
#include <string>

#include "Vec3.hpp"

namespace AprilPoseEstimator
{
    class Logger
    {
    public:
        static inline void LogPose(std::string prefix, Vec3 pose)
        {
            std::cout << prefix << ": (" << pose.x << ", " << pose.y << ", " << pose.z << ")" << std::endl;
        }

        static inline void Info(std::string a)
        {
            std::cout << "[INFO] " << a << std::endl;
        }

        static inline void Warn(std::string a)
        {
            std::cout << "[WARN] " << a << std::endl;
        }

        static inline void Error(std::string a)
        {
            std::cout << "[ERR] " << a << std::endl;
        }
    };
}
