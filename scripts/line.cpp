#include <opencv2/opencv.hpp>
#include <vector>

int equation(cv::Point2f p1, cv::Point2f p2, float& a, float& b, float& x)
{
    // a * p1.x + b = p1.y
    // a * p2.x + b = p2.y
    // <=> a * p2.x + p1.y - a * p1.x = p2.y
    // <=> a * (p2.x - p1.x) = p2.y - p1.y

    if (p1.x != p2.x) 
    {
        a = (p2.y - p1.y) / (p2.x - p1.x);
        b = p1.y - a * p1.x;
        return 0;
    }
    else
    {
        // p2.x == p1.x -> verticle line, parallel with y-axis
        x = p1.x;
        return 1;
    }

}

bool intersection(const cv::Vec4f& l1, const cv::Vec4f& l2, cv::Point2f &r)
{
    // points of l1
    const cv::Vec2f p1 = cv::Vec2f(l1[0], l1[1]);
    const cv::Vec2f p2 = cv::Vec2f(l1[2], l1[3]);

    // points of l2
    const cv::Vec2f p3 = cv::Vec2f(l2[0], l2[1]);
    const cv::Vec2f p4 = cv::Vec2f(l2[2], l2[3]);
    // y1 = a1*x1 + b1
    // y2 = a2*x2 + b2
    // y1 intersect y2 <=> y1 = y2 = y, x1 = x2 = x
    // a1*x + b1 = a2*x + b2
    // <=> (a1 - a2)*x = b2 - b1
    // <=> a1 == a2 => parallel
    // OR  x = (b2 - b1) / (a1 - a2)
    //  => y = a1 * x + b1

    float a1, b1, x1, a2, b2, x2;
    if (equation(p1, p2, a1, b1, x1) == 0)
    {
        
    }
}

void run()
{
    const int horizontalLineY = 80;

    cv::Mat image = cv::imread("image.jpg", cv::IMREAD_ANYCOLOR);
    image = image(cv::Rect(0, horizontalLineY, image.cols, image.rows - horizontalLineY));

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat edges;
    cv::Canny(gray, edges, 50, 200);

    cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_NONE);
    std::vector<cv::Vec4f> lines;
    lsd->detect(edges, lines);

    cv::Mat lineImage(image);
    lsd->drawSegments(lineImage, lines);

    cv::imshow("Edges", edges);
    cv::imshow("Lines", lineImage);

    
    // cv::Vec4f horizontalLine = cv::Vec4f(0, horizontalLineY, image.cols - 1, horizontalLineY);
    // for (int i = 0; i < lines.size(); i++)
    // {
    //     cv::Point2f intersectPoint;
    //     if (intersection(lines[i], horizontalLine, intersectPoint))
    //     {
    //         for ()
    //     }
    // }



    cv::waitKey();



    cv::destroyAllWindows();

}


int main(void)
{
    run();

    return 0;
}