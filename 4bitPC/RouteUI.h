#pragma once
#include <opencv2/opencv.hpp>
#include "CommandGenerator.h"
#include <vector>

class RouteUI {
public:
    RouteUI() {
        
        image = cv::imread("test.jpg");
        dims = image.size();
        cv::namedWindow("Route UI");
        cv::setMouseCallback("Route UI", onMouse, this);
        cmdG = CommandGenerator();
        lastRotation = 0;
    }

    std::vector<std::string> run();

private:
    cv::Size2d dims;
    cv::Mat startImage;
    cv::Mat image;
    CommandGenerator cmdG;
    std::vector<std::pair<cv::Point, int>> circles;
    cv::Point clickPoint;
    std::vector<std::pair<double, double>> Ang_Dist;
    double lastRotation; //orienteret mod højre fra start


    static void onMouse(int event, int x, int y, int flags, void* userdata) {
        RouteUI* ui = static_cast<RouteUI*>(userdata);
        ui->handleMouseEvent(event, x, y);
    }

    void handleMouseEvent(int event, int x, int y);

    std::pair<double,double> calc_Ang_Dist(const cv::Point& point1, const cv::Point& point2, double);
    void drawCircle(const cv::Point& center, int radius);
    void drawLine(const cv::Point& start, const cv::Point& end);
    void updateImage();
    void clearScreen();
    void saveFinalRoute();
    void saveSteps();
};