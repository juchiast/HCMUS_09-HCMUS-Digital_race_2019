#include <cstdlib>
#include <opencv2/highgui.hpp>
#include <string>
#include "opencv2/opencv.hpp"

#include "traffic_sign_detector.hpp"
#include <unistd.h>

#define SIZE_X 64.0

static cv::Mat blur(cv::Mat image)
{
  cv::Mat result;
  cv::GaussianBlur(image, result, cv::Size(5, 5), 0, 0);
  return result;
}

static double matching(cv::Mat image, cv::Mat templ)
{
  using namespace cv;
  Mat img_display;
  image.copyTo(img_display);

  int result_rows = image.rows - templ.rows + 1;
  int result_cols = image.cols - templ.cols + 1;
  cv::Mat result(result_rows, result_cols, CV_32FC1);

  int match_method = CV_TM_SQDIFF;
  matchTemplate(image, templ, result, CV_TM_SQDIFF);
  //   normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  /// Localizing the best match with minMaxLoc
  double minVal;
  double maxVal;
  Point minLoc;
  Point maxLoc;

  Point matchLoc;

  minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
  return minVal;
}

SignDetector::SignDetector(std::string left_image_path, std::string right_image_path)
{
  cv::Mat left_image = cv::imread(left_image_path, cv::IMREAD_GRAYSCALE);
  cv::Mat right_image = cv::imread(right_image_path, cv::IMREAD_GRAYSCALE);

  CV_Assert(!left_image.empty());
  CV_Assert(!right_image.empty());

  LEFT_TEMPLATE = blur(left_image);
  RIGHT_TEMPLATE = blur(right_image);

  MAX_DIFF = matching(LEFT_TEMPLATE, cv::Scalar::all(255) - LEFT_TEMPLATE);
}

SignDetector::~SignDetector()
{
}

cv::Mat SignDetector::deNoise(cv::Mat inputImage)
{
  cv::Mat output;
  cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);
  return output;
}

void SignDetector::detect(cv::Mat frame)
{
  using namespace cv;
  CV_Assert(!frame.empty());

  // Denoise image with gaussian blur
  cv::Mat img_denoise = deNoise(frame);
  // cv::Mat img_denoise = frame;

  cv::Mat hsvImg;
  cv::cvtColor(frame, hsvImg, cv::COLOR_BGR2HSV);

  cv::Mat binary;
  cv::inRange(hsvImg, cv::Scalar(0 / 2.0, 127, 0), cv::Scalar(360 / 2.0, 255, 255), binary);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  std::vector<std::vector<cv::Point>> contours_poly(contours.size());
  std::vector<cv::Rect> boundRect(contours.size());

  cv::Mat drawing;
  cv::cvtColor(binary, drawing, cv::COLOR_GRAY2BGR);

  double maxPercent = 0;
  size_t i_max = -1;
  int max_type = 0;

  for (size_t i = 0; i < contours.size(); i++)
  {
    approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
    boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));

    if (boundRect[i].width <= 32 && boundRect[i].height <= 24)
    {
      continue;
    }

    cv::Mat mask = Mat::zeros(binary.size(), CV_8UC1);
    drawContours(mask, contours, i, Scalar(255), CV_FILLED);
    cv::Mat crop = cv::Mat::zeros(binary.size(), CV_8UC1);
    binary.copyTo(crop, mask);

    cv::Mat resized;
    cv::resize(crop(boundRect[i]), resized, cv::Size(), SIZE_X / boundRect[i].width, SIZE_X / boundRect[i].height);
    resized = blur(resized);

    {
      double val = matching(resized, LEFT_TEMPLATE);
      double percent = 1.0 - val / MAX_DIFF;
      if (percent > maxPercent)
      {
        i_max = i;
        maxPercent = percent;
        max_type = -1;
      }
    }

    {
      double val = matching(resized, RIGHT_TEMPLATE);
      double percent = 1.0 - val / MAX_DIFF;
      if (percent > maxPercent)
      {
        i_max = i;
        maxPercent = percent;
        max_type = 1;
      }
    }
  }

  if (maxPercent >= 0.80)
  {
    cv::Scalar color = max_type == -1 ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
    cv::rectangle(drawing, boundRect[i_max], color, 2, 8, 0);
    this->signTypeDetected.id = max_type == -1 ? TrafficSign::Left : TrafficSign::Right;
    this->signTypeDetected.confident = maxPercent;
    this->signTypeDetected.boundingBox = boundRect[i_max];
  }
  else
  {
    this->signTypeDetected.id = TrafficSign::None;
  }
}

const Sign *SignDetector::getSign() const
{

  if (this->signTypeDetected.id != TrafficSign::None)
  {
    return &signTypeDetected;
  }
  return nullptr;
}
