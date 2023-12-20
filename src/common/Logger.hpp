#pragma once

#include <iostream>
#include <string>

#include "Vec3.hpp"

namespace AprilPoseEstimator
{
    class Logger
    {
    public:
        static inline void InfoVec3(std::string prefix, Vec3 pose)
        {
            std::cout << "[INFO] " << prefix << ": (" << pose.x << ", " << pose.y << ", " << pose.z << ")" << std::endl;
        }
        
        static inline void InfoDouble(std::string prefix, double dbl)
        {
            std::cout << "[INFO] " << prefix << ": " << dbl << std::endl;
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
