#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/apriltag_pose.h>

#include "PoseEstimator.hpp"
#include "common/Common.hpp"

namespace AprilPoseEstimator 
{
    apriltag_family_t* PoseEstimator::s_tf;
    apriltag_detector_t* PoseEstimator::s_td;

    void PoseEstimator::Init()
    {
        // setup april tags
        s_tf = tag16h5_create();
        s_td = apriltag_detector_create();
        apriltag_detector_add_family(s_td, s_tf);
    }

    Vec3 PoseEstimator::GetPose(const cv::Mat frame)
    {
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(AprilPoseEstimator::PoseEstimator::s_td, &im);
        
        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            if (det->id != 0) continue; // skip if tag not 0 (for testing only)
            
            // draw lines around detected tag
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
            // get focal length based of fov
            double xFocalLength = (im.width / 2) / tan((PE_CAMERA_HORIZONTAL_FOV / 2) * (PI / 180));
            double yFocalLength = (im.height / 2) / tan((PE_CAMERA_VERTICLE_FOV / 2) * (PI / 180));

            apriltag_detection_info_t info;
            info.det = det;
            info.tagsize = PE_TAG_SIZE;
            info.fx = xFocalLength;
            info.fy = yFocalLength;
            info.cx = im.width / 2;
            info.cy = im.height / 2;

            apriltag_pose_t tagPose;
            double err = estimate_tag_pose(&info, &tagPose);

            
            apriltag_detections_destroy(detections);

            // return pose of detected april tag for now
            Vec3 pose { .x = tagPose.t->data[0], .y = tagPose.t->data[1], .z = tagPose.t->data[2] };
            return pose;
        }

        Vec3 pose { .x = 0, .y = 0, .z = 0 };
        return pose;
    }
};
