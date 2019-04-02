#include <opencv2/opencv.hpp>
#include "traffic_sign_detector.hpp"
using namespace std;
using namespace cv;

static SignDetector signDetector;

void processFrame(cv::Mat frame)
{
    signDetector.detect(frame);
    

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

    processFrame(frame);

    return 0;
}