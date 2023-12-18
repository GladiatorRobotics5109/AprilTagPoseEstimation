#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/apriltag_pose.h>

#include "common/Common.hpp"
#include "PoseEstimator.hpp"
#include "common/Logger.hpp"

using namespace AprilPoseEstimator;

int main()
{
    cv::VideoCapture cap = cv::VideoCapture(0);
    cv::Mat frame;

    if (!cap.isOpened())
    {
        Logger::Error("Unable to open video capture");
        return 1;
    }

    PoseEstimator::Init();

    while (true)
    {
        cap >> frame;
        if (frame.empty())
        {
            std::cout << "frame is empty" << std::endl;
            return 1;
        }
        
        Vec3 estimatedPose = PoseEstimator::GetPose(frame);
        Logger::LogPose("tag 0 pose", estimatedPose);

        cv::imshow("Camera output", frame);
        if (cv::pollKey() == 27) // if esc pressed -> exit
            break;
    }

    return 0;
}
