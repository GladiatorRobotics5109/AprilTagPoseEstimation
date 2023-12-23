#include <opencv2/opencv.hpp>

#include "common/Logger.hpp"
#include "PoseEstimator.hpp"

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
        auto startTime = std::chrono::steady_clock::now();

        cap >> frame;
        if (frame.empty())
        {
            std::cout << "frame is empty" << std::endl;
            return 1;
        }
        
        Vec3 estimatedPose = PoseEstimator::GetPose(frame);
//        Logger::InfoVec3("camera pose", estimatedPose);

//        cv::imshow("Camera output", frame);
//        if (cv::pollKey() == 27) // if esc pressed -> exit
//            break;
        
        auto endTime = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        Logger::InfoDouble("Frame time (ms)", (double)duration.count());
    }

    return 0;
}
