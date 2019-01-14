#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "ros/console.h"
#include "signdetector.h"

#include <unistd.h>
#include <iostream>

SignDetector::SignDetector()
{
    char cwd[255];
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
        ROS_INFO("Current working dir: %s\n", cwd);
    }

    hog = cv::HOGDescriptor(cv::Size(64, 64),
                      cv::Size(32, 32),
                      cv::Size(16, 16),
                      cv::Size(32, 32),
                      9, 1, -1, 0, 0.2,
                      1, 64, 1);
    ROS_INFO_STREAM("HOG Descriptor created");

    svm = cv::ml::SVM::create();
    ROS_INFO_STREAM("Support Vector Machine Created");

    ///////// TRAINING ////////////
    trainStage(hog, svm);

    //////// CLASSIFICATION /////////
    ROS_INFO_STREAM("Sign Detection and Classification started...");
}

SignDetector::~SignDetector()
{

}

cv::Mat SignDetector::deNoise(cv::Mat inputImage) {
    cv::Mat output;
    cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);
    return output;
}

std::vector<cv::Mat> SignDetector::MSER_Features(cv::Mat binary_img) {
    cv::Mat detection;
    cv::Size size(64, 64);
    std::vector<cv::Mat> detections;

    // cv::Ptr<cv::MSER> ms = cv::MSER::create(5, 64, 500, 0.9, 0.1, 200, 1.01, 0.1, 1);
    cv::Ptr<cv::MSER> ms = cv::MSER::create();
    std::vector<std::vector<cv::Point> > regions;
    std::vector<cv::Rect> mser_bbox;
    ms->detectRegions(binary_img, regions, mser_bbox);

    // For every bounding box in the image
    for (cv::Rect i : mser_bbox) {
        // Ratio filter of detected regions
        double ratio = (static_cast<double>(i.height) /
            static_cast<double>(i.width));

        if (ratio > 0.8 && ratio < 1.2) {
            // Crop bounding boxes to get new images
            detection = binary_img(i);

            // Resize images  to fit the trained data
            cv::resize(detection, detection, size);

            // Output the vector of images
            detections.push_back(detection);
            this->boxes.push_back(i);
        }
    }
    return detections;
}

cv::Mat SignDetector::HOG_Features(cv::HOGDescriptor hog,
    std::vector<cv::Mat> imgs) {
    std::vector<std::vector<float> > HOG;

    // For all of the images of the vector, compute HOG features
    for (cv::Mat i : imgs) {
        std::vector<float> descriptor;
        hog.compute(i, descriptor);
        HOG.push_back(descriptor);
    }

    // Convert HOG features vector to Matrix for the SVM
    cv::Mat signMat(HOG.size(), HOG[0].size(), CV_32FC1);
    auto i = 0;
    while (i < HOG.size()) {
        auto j = 0;
        while (j < HOG[0].size()) {
            signMat.at<float>(i, j) = HOG[i][j];
            j++;
        }
        i++;
    }
    return signMat;
}

void SignDetector::loadTrainingImgs(const char* dirname, int label, cv::Size size, std::vector<cv::Mat> &trainImgs,
    std::vector<int> &trainLabels)
{
    cv::String pathname = dirname;
    std::vector<cv::String> filenames;
    cv::glob(pathname, filenames);

    for (cv::String i : filenames) {
        cv::Mat src = imread(i);
        cv::resize(src, src, size);
        trainImgs.push_back(src);
        trainLabels.push_back(label);
    }
}

void SignDetector::loadTrainingImgs(std::vector<cv::Mat> &trainImgs, std::vector<int> &trainLabels) {

    // Load all the turn left signs images from dataset and label them
    loadTrainingImgs("./Training_Images/1", 1, cv::Size(64,64), trainImgs, trainLabels);
    loadTrainingImgs("./Training_Images/2", 2, cv::Size(64,64), trainImgs, trainLabels);
}

void SignDetector::SVMTraining(cv::Ptr<cv::ml::SVM> &svm, cv::Mat trainHOG,
    std::vector<int> trainLabels) {
    // Set parameters of the SVM
    svm->setGamma(0.50625);
    svm->setC(12.5);
    svm->setKernel(cv::ml::SVM::RBF);
    svm->setType(cv::ml::SVM::C_SVC);

    // Feed SVM with all the labeled data and train it
    cv::Ptr<cv::ml::TrainData> td = cv::ml::TrainData::create(trainHOG,
        cv::ml::ROW_SAMPLE, trainLabels);
    svm->train(td);
}

int SignDetector::trainStage(cv::HOGDescriptor &hog, cv::Ptr<cv::ml::SVM> &svm) {
    ROS_INFO_STREAM("SVM Training Stage started...");
    // Load training data and resize
    std::vector<cv::Mat> trainImgs;
    std::vector<int> trainLabels;
    this->loadTrainingImgs(trainImgs, trainLabels);

    // HOG features of training images
    cv::Mat trainHOG = this->HOG_Features(hog, trainImgs);

    // Train SVM and save model
    this->SVMTraining(svm, trainHOG, trainLabels);
    ROS_INFO_STREAM("SVM Training Stage completed");

    // Return 1 as success
    return 1;
}

float SignDetector::SVMTesting(cv::Ptr<cv::ml::SVM> &svm, cv::Mat testHOG) {
    cv::Mat answer;

    // Feed SVM with HOG features from detections and label it
    svm->predict(testHOG, answer);

    // Return the label of the detection
    auto i = 0;
    while (i < answer.rows) {
        this->traffic_sign = answer.at<float>(i, 0);
        i++;
        return this->traffic_sign;
    }
}

void SignDetector::visualize(cv::Mat frame) {
    for (cv::Rect i : this->boxes) {
        ROS_INFO("%.2f, [%u, %u, %u, %u]", traffic_sign, i.x, i.y, i.width, i.height);
        cv::rectangle(frame, i, CV_RGB(50, 200, 0), 2);
        if (this->traffic_sign == 1) {
            cv::Point org(i.x, i.y - 5);
            cv::putText(frame, "Left", org,
                cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 255));
        }
        else if (this->traffic_sign == 2) {
            cv::Point org(i.x, i.y - 5);
            cv::putText(frame, "Right", org,
                cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 255));
        }
    }

    this->boxes.clear();
}

bool SignDetector::detect(cv::Mat frame)
{
    CV_Assert(!frame.empty());

    this->traffic_sign = 0;

    // Denoise image with gaussian blur
    cv::Mat img_denoise = deNoise(frame);

    cv::Mat hsvImg;
    cv::cvtColor(frame, hsvImg, cv::COLOR_BGR2HSV);

    cv::Mat binary;
    cv::inRange(hsvImg, cv::Scalar(160/2.0, 150,  100), cv::Scalar(210/2.0, 255, 255), binary);
    // cv::imshow("binary", binary);

    std::vector<std::vector<cv::Point> > cnts;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if (cnts.empty())
    {
        return false;
    }

    bool flag_found = false;

    for (std::vector<cv::Point> cnt : cnts)
    {
        double area = cv::contourArea(cnt);
        if (area > 40)
        {
            flag_found = true;
            break;
        }
    }

    if (!flag_found)
    {
        return false;
    }

    // cv::Mat imCnt = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3);
    // cv::drawContours(imCnt, cnts, -1, cv::Scalar(0,255,0), 2, 8, hierarchy, 0, cv::Point());
    // cv::imshow("contours", imCnt);




    // Get the detections using MSER
    // std::vector<cv::Mat> imgs_mser = MSER_Features(frame);
    std::vector<cv::Mat> imgs_mser = MSER_Features(binary);

    // If there are detection in the frame:
    if (imgs_mser.size() != 0) {
        // HOG features of detections
        cv::Mat testHOG = HOG_Features(hog, imgs_mser);

        // Evaluate using the SVM
        traffic_sign = SVMTesting(svm, testHOG);
        
        imgs_mser.clear();
    }

    return true; // even though nothing found, but should slow the car for attention
}

TrafficSign SignDetector::getTrafficSign()
{
    int trafficSignInt = static_cast<int>(traffic_sign);
    return TrafficSign(trafficSignInt);
}