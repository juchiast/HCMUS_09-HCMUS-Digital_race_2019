#include <opencv2/opencv.hpp>
#include "lane_detector.hpp"
using namespace std;
using namespace cv;

static LaneDetector laneDetector;
float len1m = 40;

cv::Mat visualize;

void approximateLeftLane(std::vector<cv::Point>& rightLane)
{
    for (int i = 0; i < rightLane.size() - 1; i++)
    {
        if (rightLane[i] == Point{0, 0} || rightLane[i+1] == Point{0, 0}) continue;
        cv::Point delta = rightLane[i+1] - rightLane[i];
        cv::Point2f perdepencular = {-delta.y, delta.x};
        float len = sqrt(delta.x*delta.x + delta.y*delta.y);
        
        perdepencular.x /= len;
        perdepencular.y /= len;

        cv::Point2f rightLaneFloat = {rightLane[i].x *1.0f, rightLane[i].y * 1.0f};
        cv::Point2f estimate = rightLaneFloat + perdepencular * len1m;
        cv::circle(visualize, estimate, 2, Scalar(0, 255, 255));
        cv::circle(visualize, rightLane[i], 2, Scalar(255, 0, 255));
    }
    cv::imshow("estimate", visualize);
    cv::waitKey(0);
}

void processFrame(cv::Mat frame)
{
    laneDetector.detect(frame);
    std::vector<cv::Point>&& leftLane = laneDetector.getLeftLane();
    std::vector<cv::Point>&& rightLane = laneDetector.getRightLane();

    // int i = rightLane.size() - 1;
    // while(i >= 0 && leftLane[i] == null && rightLane[i] == null) 
    //     i--;

    // lane1m = abs(leftLane[i].x - rightLane[i].x);

    approximateLeftLane(rightLane);
    

    // TODO: Do something here...
}

// argv[1] = filepath
int main(int argc, char** argv)
{
    //// Uncomment if reading from video
    // VideoCapture video{argv[1]};    
    // Mat frame;
    // while(1)
    // {
    //     video >> frame;
    //     if (frame.empty())
    //     {
    //         break;
    //     }

    //     processFrame(frame);
    // }
    // video.release();

    Mat frame;
    frame = cv::imread(argv[1], cv::IMREAD_ANYCOLOR);
    if (frame.empty())
    {
        std::cout << "Error file";
        return -1;
    }

    cv::pyrDown(frame, frame);
    visualize = cv::Mat::zeros(cv::Size(240, 320), CV_8UC3);


    processFrame(frame);

    return 0;
}