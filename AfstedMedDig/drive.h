#pragma once
#include <iostream>
#include <string>
#include <functional>
#include <memory>
#include <cmath>
#include <map>
#include <unistd.h>

#include "rb3_cpp_publisher.h"

class Drive {

private:
std::shared_ptr<RB3_cpp_publisher> _publisher;

//input 
public:
    Drive(std::shared_ptr<RB3_cpp_publisher> publisher);
    void forwards(std::string binaryNum);
    void backwards(std::string binaryNum);
    void turnleft(std::string binaryNum);
    void turnRight(std::string binaryNum);
};