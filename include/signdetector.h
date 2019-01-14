#include <opencv2/opencv.hpp>
#include <vector>

enum TrafficSign
{
    Slow = 0,
    Left = 1,
    Right = 2
};

class SignDetector
{
public:
    SignDetector();
    ~SignDetector();

    bool detect(cv::Mat frame);
    void visualize(cv::Mat frame);
    TrafficSign getTrafficSign();
private:
    void loadTrainingImgs(const char* dirname, int label, cv::Size size, std::vector<cv::Mat> & trainImgs, std::vector<int> &trainLabels);
    void loadTrainingImgs(std::vector<cv::Mat> &trainImgs, std::vector<int> &trainLabels);
    void SVMTraining(cv::Ptr<cv::ml::SVM> &svm, cv::Mat trainHOG, std::vector<int> trainLabels);
    std::vector<cv::Mat> MSER_Features(cv::Mat img);
    cv::Mat HOG_Features(cv::HOGDescriptor hog, std::vector<cv::Mat> imgs);
    int trainStage(cv::HOGDescriptor &hog, cv::Ptr<cv::ml::SVM> &svm);

    float SVMTesting(cv::Ptr<cv::ml::SVM> &svm, cv::Mat testHOG);

    cv::Mat deNoise(cv::Mat inputImage);

private:
    cv::HOGDescriptor hog;
    cv::Ptr<cv::ml::SVM> svm;

    std::vector<cv::Rect> boxes;  /// Bounding boxes in the current frame
    float traffic_sign;  /// The label of the detections, outputed by the SVM
    
};