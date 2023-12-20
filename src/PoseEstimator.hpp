#pragma once

#include <map>
#include <vector>

#include <opencv2/opencv.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/apriltag_pose.h>

#include "common/Vec3.hpp"
#include "common/Common.hpp"

namespace AprilPoseEstimator
{
    class PoseEstimator
    {
    public:
        static void Init();
        static Vec3 GetPose(const cv::Mat frame);
    private:
        static apriltag_family_t* s_tf;
        static apriltag_detector_t* s_td;

        static std::map<int, Vec3> s_apriltagPoses;
    };
}
