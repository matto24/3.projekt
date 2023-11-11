#include "drive.h"
#include <chrono>

Drive::Drive(std::shared_ptr<RB3_cpp_publisher> publisher): _publisher(publisher){}

//Translational velocity is in m/s with a max of 0.22 m/s
//Rotational velocity is in rad/s with a max of 2.84 rad/s, equivalent to 162.72 deg/s

void Drive::forwards(std::string binaryNum) {
    //Input is the distance in 1/10 m
    //Converting input to integer
    int length = stoi(binaryNum.substr(0,binaryNum.size()), nullptr, 2);

    //Converting to meters
    double distInMeters = length/10;
    //Time the robot should drive for
    double time = distInMeters/0.2;

    //Publishing vel = 0.2 m/s
    _publisher->publish_vel(0.2,0);

    //Adding delay according to distance
    sleep_for(seconds(time));
    //Set velocity back to 0
     _publisher->publish_vel(0,0);
}

void Drive::backwards(std::string binaryNum) {
        //Input is the length in m
    //Converting input to integer
    int length = stoi(binaryNum.substr(0,binaryNum.size()), nullptr, 2);

    double distInMeters = length/10;

    //Time the robot should drive for
     double time = distInMeters/0.2;

    //Publishing vel = -0.2 m/s
    _publisher->publish_vel(-0.2,0);

    //Adding delay according to distance
    sleep_for(seconds(time));
    //Set velocity back to 0
     _publisher->publish_vel(0,0);
}

//To rotate with a rotational vel of 90deg/s the ang vel command should be set to 1.57
// x*180/pi = 90, x = 1.57
void Drive::turnleft(std::string binaryNum) {
    int degree = stoi(binaryNum.substr(0,binaryNum.size()), nullptr, 2);

    double time = degree/90;

    _publisher->publish_vel(0, 1.57);
    sleep_for(seconds(time));
    //Set velocity back to 0
     _publisher->publish_vel(0,0);

}

void Drive::turnRight(std::string binaryNum) {
    int degree = stoi(binaryNum.substr(0,binaryNum.size()), nullptr, 2);

    double time = degree/90;

    _publisher->publish_vel(0,-1.57);

    sleep_for(seconds(time));
    //Set velocity back to 0
    _publisher->publish_vel(0,0);
}

