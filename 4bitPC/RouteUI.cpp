#include "RouteUI.h"
#include "cmath"

    std::vector<std::string> RouteUI::run() {
    std::vector<std::string> out;
        while (true) {
            cv::imshow("Route UI", image);
            char key = cv::waitKey(10);
            if(key == 32) { // Clearing on spacebar
                clearScreen();
            }

            if(key==13){ // Enter to send route
                
                for(std::pair<double,double> ad : Ang_Dist){
                    
                    //std::cout << i << std::endl;
                    if(ad.second > 0){
                        out.push_back(cmdG.createCommand(1,std::abs(ad.second * (180/M_PI))));
                        std::cout << "Drej til højre med: " << std::abs(ad.second * (180/M_PI)) << " grader -> " << cmdG.createCommand(1,std::abs(ad.second * (180/M_PI))) << std::endl;
                    }
                    else if(ad.second < 0){
                        out.push_back(cmdG.createCommand(2,std::abs(ad.second * (180/M_PI))));
                        std::cout << "Drej til venstre med: " << std::abs(ad.second * (180/M_PI)) << " grader -> " << cmdG.createCommand(2,std::abs(ad.second * (180/M_PI))) << std::endl;
                    }
                    if(std::abs(ad.first) > 0){
                        double distance = abs(ad.first);
                        int lastdrive = 0; //to keep it from driving 255 back to back
                        while(distance > 0){
                            if(distance > 255){
                                if(lastdrive == 255){
                                    out.push_back(cmdG.createCommand(3,std::abs(254)));
                                    distance -= 254;
                                    lastdrive = 254;
                                    std::cout << "Kør frem i: " << std::abs(254) << " decimeter -> " << cmdG.createCommand(3,std::abs(254)) << std::endl;
                                }
                                else{
                                    lastdrive = 255;
                                    out.push_back(cmdG.createCommand(3,std::abs(255)));
                                    distance -= 255;
                                    std::cout << "Kør frem i: " << std::abs(255) << " decimeter -> " << cmdG.createCommand(3,std::abs(255)) << std::endl;
                                }
                                
                            }
                            else{
                                out.push_back(cmdG.createCommand(3,std::abs(distance)));
                                std::cout << "Kør frem i: " << std::abs(distance) << " decimeter -> " << cmdG.createCommand(3,std::abs(distance)) << std::endl;
                                distance = 0;
                            }
                            
                        }
                        
                    }
                }
                if(out.size() > 0){
                    cv::destroyAllWindows();
                    saveFinalRoute();
                    out.push_back(cmdG.createCommand(0,0));
                    return out;
                }
            }
            if (key == 8){  // For deleting last checkpoint
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
        double ang;
        ang = atan2(dis_y,dis_x)-prev;
        if(ang > M_PI){
            ang -= 2*M_PI;
        }
        //return {(sqrt(dis_x * dis_x + dis_y * dis_y))/10, atan2(dis_y,dis_x)-prev};
        return {1414.2*(sqrt(dis_x * dis_x + dis_y * dis_y)/sqrt(dims.height*dims.height+dims.width*dims.width)), ang};

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
            lastRotation += angleDistance.second;
            if(lastRotation > M_PI){
              lastRotation -= 2*M_PI;
            }
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
        lastRotation = 0;
    }
