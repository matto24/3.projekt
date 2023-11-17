#include "RouteUI.h"
#include "cmath"



    std::vector<std::string> RouteUI::run() {
    std::vector<std::string> out;
        while (true) {
            
            cv::imshow("Route UI", image);
            char key = cv::waitKey(10);
            if(key == 32) {
                clearScreen();

            }

            if(key==13){
                
                for(std::pair<double,double> ad : Ang_Dist){
                    
                    //std::cout << i << std::endl;
                    if(ad.second > 0){
                        out.push_back(cmdG.createCommand(42,std::abs(ad.second * (180/M_PI))));
                        //std::cout << "Drej til højre med: " << std::abs(ad.second * (180/M_PI)) << " grader -> " << cmdG.createCommand(42,std::abs(ad.second * (180/M_PI))) << std::endl;
                    }
                    else if(ad.second < 0){
                        out.push_back(cmdG.createCommand(46,std::abs(ad.second * (180/M_PI))));
                        //std::cout << "Drej til venstre med: " << std::abs(ad.second * (180/M_PI)) << " grader -> " << cmdG.createCommand(46,std::abs(ad.second * (180/M_PI))) << std::endl;
                    }
                    if(std::abs(ad.first) > 0){
                        out.push_back(cmdG.createCommand(62,std::abs(ad.first)));
                        //std::cout << "Kør frem i: " << std::abs(ad.first) << " pixel -> " << cmdG.createCommand(62,std::abs(ad.first)) << std::endl;
                    }
                }
                if(out.size() > 0){
                    cv::destroyAllWindows();
                    saveFinalRoute();
                    return out;
                }
            }
            if (key == 8){
                if(circles.size() > 0){
                    circles.pop_back();
                    Ang_Dist.pop_back();
                    updateImage();
                }
            }

            if (key == 27)  // Exit on Esc key press
                break;
        }
        return {};
    }

    void RouteUI::handleMouseEvent(int event, int x, int y) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            
            clickPoint = cv::Point(x, y);
            circles.emplace_back(clickPoint, 10); 
            saveSteps();
            updateImage();
        }
    }


    std::pair<double,double> RouteUI::calc_Ang_Dist(const cv::Point& point1, const cv::Point& point2, double prev) {
        double dis_x = point2.x - point1.x;
        double dis_y = point2.y - point1.y;
        return {sqrt(dis_x * dis_x + dis_y * dis_y)/800, atan2(dis_y,dis_x)-prev};
    }

   
    void RouteUI::drawCircle(const cv::Point& center, int radius) {
        cv::circle(image, center, radius, cv::Scalar(0, 0, 0), -1);
    }

    void RouteUI::drawLine(const cv::Point& start, const cv::Point& end) {
        cv::line(image, start, end, cv::Scalar(0, 0, 255), 2);
    }

    void RouteUI::updateImage() {
        //image = cv::Mat::ones(800, 800, CV_8U)*255;
        image = cv::imread("test.jpg");
        double lastRotation = 0;
        for (int i = 0; i < circles.size(); i++) {
            
            drawCircle(circles[i].first, circles[i].second);

            if (i + 1 < circles.size()) {
                drawLine(circles[i].first, circles[i + 1].first);
                }
                
        }
    }

    void RouteUI::saveSteps(){
        if(circles.size() > 1){
            std::pair<double, double> angleDistance = calc_Ang_Dist(circles[circles.size()-2].first, circles[circles.size()-1].first, lastRotation);
            Ang_Dist.push_back(angleDistance);
            lastRotation = angleDistance.second;
        }
        
    }

    void RouteUI::saveFinalRoute(){
        cv::imwrite("FinalRoute.jpg", image);
        
    }


    void RouteUI::clearScreen(){
        //image = cv::Mat::ones(800, 800, CV_8U)*255;
        image = cv::imread("test.jpg");
        Ang_Dist.clear();
        circles.clear();
    }
