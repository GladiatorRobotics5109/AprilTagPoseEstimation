#include <iostream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>


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
        std::cout << "read camera" << std::endl;

	cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
	image_u8_t im = { .width = gray.cols,
		.height = gray.rows,
		.stride = gray.cols,
		.buf = gray.data
	};

	zarray_t *detections = apriltag_detector_detet(td, &im);

	// Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(frame, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
        }
	apriltag_detections_destroy(detections);
        cv::imshow("Camera output", frame);
	if(cv::waitKey(30) >= 0)
		break;
    }

    return 0;
}
