#include "navigation.hpp"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

using namespace cv;

static cv::Point null = cv::Point{};
const cv::Point Navigation::distanceNearLaneLine{10, 0};
const cv::Point Navigation::carPos{120, 300};

int Navigation::MIN_VELOCITY = 5;
int Navigation::MAX_VELOCITY = 30;
int Navigation::DEF_VELOCITY = 8;

static const int OBJ_NONE = 0;
static const int OBJ_LEFT = 1;
static const int OBJ_RIGHT = 2;

enum Sign
{
    NONE = -1,
    SLOW = 0,
    LEFT = 1,
    RIGHT = 2
};

static bool isLaneNull(const Navigation::Lane& lane)
{
    return std::all_of(lane.begin(), lane.end(), [&lane](cv::Point point){
        return point == null;
    });
}

Navigation::Navigation()
    : sign{-1}, currentSpeed(DEF_VELOCITY), currentSteer{0.0f}
    , carDir(0, 1), theta{0.0f}
    , objectDir{0}
    , visualizeImage(cv::Size(240, 320), CV_8UC3)
{
}

float Navigation::errorAngle(const Point &dst)
{
    if (dst.x == carPos.x)
        return 0;
    if (dst.y == carPos.y)
        return (dst.x < carPos.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y;
    if (dx < 0)
        return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

void Navigation::update(Lane &leftLane, Lane &rightLane)
{
    if (isLaneNull(leftLane) && isLaneNull(rightLane))
    {
        // LANE NOT FOUND!!!!
        ROS_INFO("LANE NOT FOUND!");
        return;
    } 

    auto approx_left = approximateLeftLane(rightLane);
    auto approx_right = approximateRightLane(leftLane);

    for (int i = 0; i < leftLane.size(); i++)
    {
        if (leftLane[i] == null) {
            leftLane[i] = approx_left[i];
        }
    }

    for (int i = 0; i < rightLane.size(); i++)
    {
        if (rightLane[i] == null) {
            rightLane[i] = approx_right[i];
        }
    }

    if (!isLaneNull(leftLane) && !isLaneNull(rightLane))
    {
        this->leftLane = leftLane;
        this->rightLane = rightLane;
    } 
    
    if (isLaneNull(leftLane))
    {
        this->leftLane = approximateLeftLane(rightLane);
        this->rightLane = rightLane;
        ROS_INFO("approx left!");
    } else if (isLaneNull(rightLane))
    {
        this->leftLane = leftLane;
        this->rightLane = approximateRightLane(leftLane);
        ROS_INFO("approx right!");
    }

    // this->leftLane = leftLane;
    // this->rightLane = rightLane;
}

void Navigation::update(const TurningFlags &leftTurn, const TurningFlags &rightTurn)
{
    this->leftTurn = leftTurn;
    this->rightTurn = rightTurn;
}

void Navigation::update(const SignDetected &sign)
{
    if (this->skipFrameCount <= 0)
    {
        this->sign = sign.id;
    }
}

float Navigation::getSpeed()
{
    if (isTurning())
    {
        this->currentSpeed = MIN_VELOCITY;
    }
    else
    {
        this->currentSpeed = DEF_VELOCITY;
    }
    return this->currentSpeed;
}

float Navigation::getSteer()
{
    // if (isTurning())
    // {
    //     return this->getSteerTurning();
    // }

    if (this->sign == LEFT)
    {
        turnLeft();
    }
    else if (this->sign == RIGHT)
    {
        turnRight();

    }
    else
    {
        forward();
    }

    return this->currentSteer;
}

float Navigation::getSteerTurning()
{
    float steeringAngle = 0;
    if (this->sign == LEFT)
    {
        steeringAngle = skipFrameCount * 5;
    }
    else if (this->sign == RIGHT)
    {
        steeringAngle = -skipFrameCount * 5;
    }
    skipFrameCount--;
    return steeringAngle;
}

void Navigation::turnRight()
{
    int i = rightLane.size() - 1;
    while (i >= 0 && rightLane[i] == null) i--;
    if (i < 0)
    {
        return;
    }


    // leftLane = approximateLeftLane(rightLane);
    // currentSteer = errorAngle((leftLane[i] + rightLane[i])/2);


    // if (!rightTurn[i])
    // {
    //     forward();
    // }
    // else
    // {
        currentSteer = errorAngle(rightLane[i] - distanceNearLaneLine);
    // }
}

void Navigation::turnLeft()
{
    int i = leftLane.size() - 1;
    while (i >= 0 && leftLane[i] == null) i--;
    if (i < 0)
    {
        return;
    }

    // rightLane = approximateRightLane(leftLane);

    
    // currentSteer = errorAngle((leftLane[i] + rightLane[i])/2);

    // if (!leftTurn[i])
    // {
    //     forward();
    // }
    // else
    // {
        currentSteer = errorAngle(leftLane[i] + distanceNearLaneLine);
    // }
}

void Navigation::forward()
{
    int i = rightLane.size() - 3;
    if (i < 0)
    {
        return;
    }

    while (leftLane[i] == null && rightLane[i] == null)
    {
        i--;
        if (i < 0)
            return;
    }
    if (leftLane[i] != null && rightLane[i] != null)
    {
        // currentSteer = errorAngle((leftLane[i] + rightLane[i]) / 2);
        // default is turn right if turnable
        // if (rightTurn[i])
        // {
        //     currentSteer = errorAngle(rightLane[i] - distanceNearLaneLine);
        // }
        // else
        if (objectDir == NONE)
        {
            currentSteer = errorAngle((leftLane[i] + rightLane[i]) / 2);
        } else if (objectDir == RIGHT)
        {
            // use left lane if obj on the right
            currentSteer = errorAngle(leftLane[i] + distanceNearLaneLine);
        } else if (objectDir == LEFT)
        {
            // use right lane if obj on the left
            currentSteer = errorAngle(rightLane[i] - distanceNearLaneLine);
        }
    }
    else if (leftLane[i] != null)
    {
        currentSteer = errorAngle(leftLane[i] + distanceNearLaneLine);
    }
    else
    {
        currentSteer = errorAngle(rightLane[i] - distanceNearLaneLine);
    }
}

bool Navigation::isTurning() const
{
    return (this->sign == Sign::SLOW || this->sign == Sign::LEFT || this->sign == Sign::RIGHT);
}

void Navigation::updateObjectDirection(const int& direction)
{
    this->objectDir = direction;
}

Navigation::Lane Navigation::approximateLeftLane(const Navigation::Lane& rightLane)
{
    Navigation::Lane lane(rightLane.size(), null);
    for (int i = 0; i < rightLane.size() - 1; i++)
    {
        if (rightLane[i] == null || rightLane[i+1] == null) continue;
        cv::Point delta = rightLane[i+1] - rightLane[i];
        cv::Point2f perdepencular(-delta.y, delta.x);
        float len = sqrt(delta.x*delta.x + delta.y*delta.y);
        
        perdepencular.x /= len;
        perdepencular.y /= len;

        cv::Point2f rightLaneFloat = {rightLane[i].x *1.0f, rightLane[i].y * 1.0f};
        cv::Point2f estimate = rightLaneFloat + perdepencular * LANE_WIDTH;
        cv::Point estimateRound(std::round(estimate.x),  std::round(estimate.y));
        lane[i] = estimate;
    }
    return lane;
}

Navigation::Lane Navigation::approximateRightLane(const Navigation::Lane& leftLane)
{
    Navigation::Lane lane(leftLane.size(), null);
    for (int i = 0; i < leftLane.size() - 1; i++)
    {
        if (leftLane[i] == null || leftLane[i+1] == null) continue;
        cv::Point delta = leftLane[i+1] - leftLane[i];
        cv::Point2f perdepencular(delta.y, -delta.x);
        float len = sqrt(delta.x*delta.x + delta.y*delta.y);
        
        perdepencular.x /= len;
        perdepencular.y /= len;

        cv::Point2f leftLaneFloat = {leftLane[i].x *1.0f, leftLane[i].y * 1.0f};
        cv::Point2f estimate = leftLaneFloat + perdepencular * LANE_WIDTH;
        cv::Point estimateRound(std::round(estimate.x),  std::round(estimate.y));
        lane[i] = estimate;
    }
    return lane;
}

void Navigation::visualize()
{
    visualizeImage.setTo(cv::Scalar(0,0,0));
    for (auto point : leftLane) // left is blue
    {
        cv::circle(visualizeImage, point, 2, cv::Scalar(255,0,0));
    }
    for (auto point : rightLane) // right is red
    {
        cv::circle(visualizeImage, point, 2, cv::Scalar(0,0,255));
    }
    cv::imshow("Lane Navigation", visualizeImage);
    cv::waitKey(1);
}