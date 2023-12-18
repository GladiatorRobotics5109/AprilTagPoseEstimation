#pragma once

#include <memory>

#include <opencv2/opencv.hpp>

#include "common/Vec3.hpp"

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
    };
}