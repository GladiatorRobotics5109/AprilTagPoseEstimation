#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/apriltag_pose.h>

#define PI 3.1415926535897

#define CAMERA_HORIZONTAL_FOV 90
#define CAMERA_VERTICLE_FOV 50.625
#define TAG_SIZE 0.1524

int main()
{
    cv::VideoCapture cap = cv::VideoCapture(0);
    cv::Mat frame, gray;

    if (!cap.isOpened())
    {
        std::cout << "could not open video capture" << std::endl;
        return 1;
    }

    // setup april tags
    apriltag_family_t *tf = tag16h5_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    while (true)
    {
        cap >> frame;
        if (frame.empty())
        {
            std::cout << "frame is empty" << std::endl;
            return 0;
        }

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };
        zarray_t *detections = apriltag_detector_detect(td, &im);
        
        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            if (det->id != 0) continue; // skip if tag not 0 (for testing only)

            line(frame, 
                cv::Point(det->p[0][0], det->p[0][1]), 
                cv::Point(det->p[1][0], det->p[1][1]), 
                cv::Scalar(0, 0xff, 0), 2);
            
            line(frame, 
                cv::Point(det->p[0][0], det->p[0][1]), 
                cv::Point(det->p[3][0], det->p[3][1]), 
                cv::Scalar(0, 0, 0xff), 2);
            
            line(frame, 
                cv::Point(det->p[1][0], det->p[1][1]), 
                cv::Point(det->p[2][0], det->p[2][1]), 
                cv::Scalar(0xff, 0, 0), 2);
            
            line(frame, 
                cv::Point(det->p[2][0], det->p[2][1]), 
                cv::Point(det->p[3][0], det->p[3][1]), 
                cv::Scalar(0xff, 0, 0), 2);

            std::string text = std::to_string(det->id);
            
            int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            
            cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2, &baseline);
            
            cv::putText(frame, 
                        text, 
                        cv::Point(
                            det->c[0]-textsize.width/2, 
                            det->c[1]+textsize.height/2), 
                        fontface, 
                        fontscale, 
                        cv::Scalar(0xff, 0x99, 0), 
                        2);

            // estimate position of tag
            // TAG SIZE: .1524mx.1524m
            // mm -> pixels
            double xFocalLength = (im.width / 2) / tan((CAMERA_HORIZONTAL_FOV / 2) * (PI / 180));
            double yFocalLength = (im.height / 2) / tan((CAMERA_VERTICLE_FOV / 2) * (PI / 180));


            apriltag_detection_info_t info;
            info.det = det;
            info.tagsize = TAG_SIZE;
            info.fx = xFocalLength;
            info.fy = yFocalLength;
            info.cx = im.width / 2;
            info.cy = im.height / 2;

            apriltag_pose_t pose;
            double err = estimate_tag_pose(&info, &pose);

            std::cout << "estimated pose: (";
            std::cout << pose.t->data[0] << ", ";
            std::cout << pose.t->data[1] << ", ";
            std::cout << pose.t->data[3] << ")" << std::endl;

        }
        
        apriltag_detections_destroy(detections);
        
        cv::imshow("Camera output", frame);
        if (cv::pollKey() == 27) // if esc pressed exit
            break;
    }

    return 0;
}
