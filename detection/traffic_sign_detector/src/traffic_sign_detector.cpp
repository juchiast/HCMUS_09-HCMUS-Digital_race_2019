#include <cstdlib>
#include <string>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"

#include "traffic_sign_detector.hpp"
#include "traffic_sign_recognizer.hpp"

#include <unistd.h>

cv::RNG rng(12345);

SignDetector::SignDetector(SignRecognizer *recognizer)
    : signRecognizer{recognizer}
{
}

SignDetector::~SignDetector()
{
    delete signRecognizer;
}

cv::Mat SignDetector::deNoise(cv::Mat inputImage)
{
    cv::Mat output;
    cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);
    return output;
}

// std::vector<cv::Mat> SignDetector::MSER_Features(cv::Mat binary_img)
// {
//     cv::Mat detection;
//     std::vector<cv::Mat> detections;

//     cv::Ptr<cv::MSER> ms = cv::MSER::create();
//     std::vector<std::vector<cv::Point>> regions;
//     std::vector<cv::Rect> mser_bbox;
//     ms->detectRegions(binary_img, regions, mser_bbox);

//     // For every bounding box in the image
//     for (cv::Rect i : mser_bbox)
//     {
//         // Ratio filter of detected regions
//         double ratio = (static_cast<double>(i.height) /
//                         static_cast<double>(i.width));

//         if (ratio > 0.8 && ratio < 1.2)
//         {
//             // Crop bounding boxes to get new images
//             detection = binary_img(i);

//             // Output the vector of images
//             detections.push_back(detection);
//             this->boxes.push_back(i);
//         }
//     }
//     return detections;
// }

void SignDetector::detect(cv::Mat frame)
{
    signs.clear();
    CV_Assert(!frame.empty());

    // Denoise image with gaussian blur
    // cv::Mat img_denoise = deNoise(frame);
    cv::Mat img_denoise = frame;

    cv::Mat hsvImg;
    cv::cvtColor(frame, hsvImg, cv::COLOR_BGR2HSV);

    cv::Mat binary;
    cv::inRange(hsvImg, cv::Scalar(160 / 2.0, 150, 100), cv::Scalar(210 / 2.0, 255, 255), binary);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if (contours.empty())
    {
        return;
    }

    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());


    cv::Mat drawing;
    cv::cvtColor(binary, drawing, cv::COLOR_GRAY2BGR);

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i], false);
        if (area > 250) // at least 50x50
        {
            approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
            boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));

            double ratio = boundRect[i].width * 1.0 / boundRect[i].height;
            if (ratio > 0.8 && ratio < 1.2)
            {
                TrafficSign signId;
                double confident;
                signRecognizer->recognize(binary(boundRect[i]), signId, confident);

                Sign signObject;
                signObject.id = static_cast<int>(signId);
                signObject.confident = confident;
                signObject.boundingBox = boundRect[i];

                signs.push_back(signObject);

                // visualize bounding box
                cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
                cv::rectangle(drawing, boundRect[i], color, 2, 8, 0);
            }
        }
    }

    cv::imshow("Drawing....", drawing);
}

const Sign *SignDetector::getSign() const
{
    Sign* signResult = nullptr;

    if (!signs.empty())
    {
        float maxConfident = 0;
        int maxId = -1;

        for (size_t i = 0; i < signs.size(); i++)
        {
            if (signs[i].id != TrafficSign::None && signs[i].confident > maxConfident)
            {
                maxConfident = signs[i].confident;
                maxId = i;
            }
        }

        if (maxId == -1)
        {
            return nullptr;
        }
        return &this->signs[maxId];
    }

    return signResult;
}

void SignDetector::toSignMessage(cds_msgs::SignDetected &msg)
{
    const Sign *sign = getSign();
    if (sign == nullptr)
    {
        msg.id = TrafficSign::None;
    }
    else
    {
        msg.id = static_cast<int>(sign->id);
        msg.x = sign->boundingBox.x;
        msg.y = sign->boundingBox.y;
        msg.width = sign->boundingBox.width;
        msg.height = sign->boundingBox.height;
    }
}