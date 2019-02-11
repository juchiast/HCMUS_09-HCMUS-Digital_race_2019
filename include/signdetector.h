#include <opencv2/opencv.hpp>
#include <vector>
#include <team405/Sign.h>

enum TrafficSign
{
    Left = 1,
    Right = 2
};

class SignDetector
{
public:
    SignDetector();
    ~SignDetector();

    void detect(cv::Mat frame);
    void visualize(cv::Mat frame);
    std::vector<team405::Sign> getTrafficSign();

private:
    std::vector<cv::Rect> MSER_Features(cv::Mat img);
    cv::Mat deNoise(cv::Mat inputImage);

private:
    std::vector<team405::Sign> signs;  /// Bounding boxes in the current frame
};