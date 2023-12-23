#include "Vec3.hpp"

namespace AprilPoseEstimator
{
    Vec3::Vec3()
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }
    
    Vec3::Vec3(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Vec3::Vec3(apriltag_pose_t pose)
    {
        this->x = pose.t->data[0];
        this->y = -pose.t->data[1];
        this->z = pose.t->data[2];
    }

    Vec3::~Vec3()
    {

    }

    Vec3 operator +(Vec3 a, Vec3 b)
    {
        return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
    }
    
    Vec3 operator -(Vec3 a, Vec3 b)
    {
        return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    Vec3 operator *(Vec3 a, Vec3 b)
    {
        return Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
    }

    Vec3 operator /(Vec3 a, Vec3 b)
    {
        return Vec3(a.x / b.x, a.y / b.y, a.z / b.z);
    }
}