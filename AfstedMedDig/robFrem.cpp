#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <portaudio.h>
#include <fftw3.h>
#include <cmath>
#include <map>
#include <unistd.h>

#include "portaudio.h"
#include "rb3_cpp_publisher.h"
#include "drive.h"

using namespace std;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto rb3_publisher = std::make_shared<RB3_cpp_publisher>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(rb3_publisher);

    Drive robo(rb3_publisher);
    robo.forwards("11");
    robo.backwards("11");
    //90 grader venstre
    robo.turnleft("01011010");
    robo.turnRight("01011010");

    return 0;
}
