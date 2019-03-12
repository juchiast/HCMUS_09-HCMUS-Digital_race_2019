#include <opencv2/opencv.hpp>
#include <vector>

enum TrafficSign
{
    Slow = 0,
    Left = 1,
    Right = 2
};

struct Sign
{
    int id;
    int x, y, width, height;
    float confident;
};

class SignDetector
{
public:
    SignDetector();
    ~SignDetector();

    void detect(cv::Mat frame);
    std::vector<Sign> getDetections() const;
    void visualize(cv::Mat frame) const;

private:
    std::vector<cv::Mat> MSER_Features(cv::Mat img);
    TrafficSign recognize(cv::Mat boundingBox);
    cv::Mat deNoise(cv::Mat inputImage);

private:
    std::vector<cv::Rect> boxes;  /// Bounding boxes in the current frame
    std::vector<Sign> signDetections;
};