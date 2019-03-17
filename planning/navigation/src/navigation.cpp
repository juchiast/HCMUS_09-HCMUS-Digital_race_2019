#include "navigation.hpp"

using namespace cv;

static cv::Point null = cv::Point{};
const cv::Point Navigation::distanceNearLaneLine{10, 0};
const cv::Point Navigation::carPos{120, 300};

enum Sign
{
    NONE = -1,
    SLOW = 0,
    LEFT = 1,
    RIGHT = 2
};

Navigation::Navigation()
    : sign{-1}, currentSpeed{DEF_VELOCITY}, currentSteer{0.0f}
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

void Navigation::update(const Lane &leftLane, const Lane &rightLane)
{
    this->leftLane = leftLane;
    this->rightLane = rightLane;
}

void Navigation::update(const TurningFlags &leftTurn, const TurningFlags &rightTurn)
{
    this->leftTurn = leftTurn;
    this->rightTurn = rightTurn;
}

void Navigation::update(const cds_msgs::SignDetected &sign)
{
    this->sign = sign.id;
}

float Navigation::getSpeed()
{
    if (isTurning())
    {
        this->currentSpeed = MIN_VELOCITY;
    }
    else
    {
        this->currentSpeed = 30;
    }
    return this->currentSpeed;
}

float Navigation::getSteer()
{
    // if (isTurning)
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
    // TODO: implement steering when turning
    return 0.0f;
}

void Navigation::turnRight()
{
    int i = rightTurn.size() - 5;
    if (i < 0)
    {
        return;
    }

    if (!rightTurn[i])
    {
        forward();
    }
    else
    {
        currentSteer = errorAngle(rightLane[i] + distanceNearLaneLine);
    }
}

void Navigation::turnLeft()
{
    int i = leftLane.size() - 5;
    while (i > 0 && leftLane[i] == null)
    {
        i--;
        if (i < 0)
            return;
    }
    currentSteer = errorAngle(leftLane[i] + distanceNearLaneLine);
    // currentSteer = errorAngle(rightLane[i] - Point(LANE_WIDTH / 2, 0));

    // error = errorAngle(right[i] - distanceNearLaneLine);
}

void Navigation::forward()
{
    int i = leftLane.size() - 5;
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
        currentSteer = errorAngle((leftLane[i] + rightLane[i]) / 2);
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
    return (this->sign == Sign::LEFT || this->sign == Sign::RIGHT);
}