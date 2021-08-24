#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

int point_num = 4;
std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < point_num) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void draw(cv::Mat &window, const cv::Point2f &point)
{
    cv::Point2f mid(int(point.x) + 0.5, int(point.y) + 0.5);
    double dis = cv::norm(mid-point);
    for (int i = -1; i < 2; i++)
    {
        for (int j = -1; j < 2; j++)
        {
            cv::Point2f center(mid.x + i, mid.y + j);
            double res = cv::norm(center-point);
            //std::cout << res << " " << dis << "\n";
            if (res < 1)
                window.at<cv::Vec3b>(center.y, center.x)[1] += (1 - res) *255.0;
            //std::cout << res << "\n";
        }
    }
    window.at<cv::Vec3b>(int(point.y)+0.5, int(point.x)+0.5)[1] = 255;
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1)
        return control_points[0];
    std::vector<cv::Point2f> next_control_points;
    for (int i = 0; i < control_points.size()-1; i++)
    {
        auto point = t * control_points[i] + (1 - t) * control_points[i+1];
        next_control_points.push_back(point);
    }
    return recursive_bezier(next_control_points, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t);
        draw(window, point);
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == point_num) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
