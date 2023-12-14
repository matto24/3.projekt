#include "drive.h"
#include <chrono>
#include <iostream>
#include <vector>
Drive::Drive(std::shared_ptr<RB3_cpp_publisher> publisher): _publisher(publisher){}

//Translational velocity is in m/s with a max of 0.22 m/s
//Rotational velocity is in rad/s with a max of 2.84 rad/s, equivalent to 162.72 deg/s

void Drive::commands(std::vector<std::pair<int, std::string>> inputCommands) {

  for(int i=0; i<inputCommands.size(); i++) {
    switch (inputCommands[i].first) 
    {
    case 0b01: //Command-code
        turnRight(inputCommands[i].second);
        break;
    case 0b10:
        turnleft(inputCommands[i].second);
        break;
    case 0b11: //Command-code
        forwards(inputCommands[i].second);
        break;
    case 0b00:
        break;
    default:
        break;
    }
  }
}

void Drive::forwards(std::string binaryNum) {
    //Input is the distance in 1/10 m
    //Converting input to integer
    double length = stoi(binaryNum.substr(0,binaryNum.size()), nullptr, 2);

    //Converting to meters
    double distInMeters = length/10;
    std::cout << "fremad i m "<< distInMeters << std::endl;
    //Time the robot should drive for
    int time = (distInMeters/0.2)*1000;
    //Publishing vel = 0.2 m/s
    _publisher->publish_vel(0.2,0);

    //Adding delay according to distance
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    //Set velocity back to 0
     _publisher->publish_vel(0,0);
   std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

       
//To rotate with a rotational vel of 90deg/s the ang vel command should be set to 1.57
// x*180/pi = 90, x = 1.57
void Drive::turnleft(std::string binaryNum) {
    double degree = stoi(binaryNum.substr(0,binaryNum.size()), nullptr, 2);
     std::cout << "vinkel den drejer " << degree << std::endl;

    int time = (degree/90)*1000;

    _publisher->publish_vel(0, 1.57);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    //Set velocity back to 0
     _publisher->publish_vel(0,0);
   std::this_thread::sleep_for(std::chrono::milliseconds(500));

}

void Drive::turnRight(std::string binaryNum) {
    double degree = stoi(binaryNum.substr(0,binaryNum.size()), nullptr, 2);
    std::cout << "vinkel den drejer " << degree << std::endl;
    int time = (degree/90)*1000;

    _publisher->publish_vel(0,-1.57);

    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    //Set velocity back to 0
    _publisher->publish_vel(0,0);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

