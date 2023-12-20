#pragma once

#include <vector>

#include <apriltag/apriltag_pose.h>

namespace AprilPoseEstimator
{
    class Vec3 {
    public:
        Vec3();
        Vec3(double x, double y, double z);
        Vec3(apriltag_pose_t pose);
        ~Vec3();

        double x;
        double y;
        double z;
    };

    Vec3 operator +(Vec3 a, Vec3 b);
    Vec3 operator -(Vec3 a, Vec3 b);
    Vec3 operator *(Vec3 a, Vec3 b);
    Vec3 operator /(Vec3 a, Vec3 b);
}