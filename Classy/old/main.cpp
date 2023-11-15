#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <portaudio.h>
#include <fftw3.h>
#include <cmath>
#include "FFT.h"

//#include "rclcpp/rclcpp.hpp"
//#include "geometry_msgs/msg/twist.hpp"
//#include "portaudio.h"

#define SAMPLE_RATE 8000
#define RECORDING_DURATION_SECONDS 0.2 // resolution = (sample_rate /(sample_rate*duration))
#define FRAMES_PER_BUFFER 1600
#define NUM_CHANNELS 1

using namespace std;

bool shutdown = false;
/*
class RB3_cpp_publisher : public rclcpp::Node
{
public:
    // Create publisher for publishing veloctiy commands to topic "cmd_vel"
    RB3_cpp_publisher()
        : Node("rb3_cpp_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    // Class function to be called when you want to publish velocity commands
    void publish_vel(float lin_vel_cmd, float ang_vel_cmd)
    {
        // Set angular velocity to desired value (ie. turning)
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = ang_vel_cmd;

        // Set linear velocity to desired value (ie. forward and backwards)
        msg.linear.x = lin_vel_cmd;
        msg.linear.y = 0;
        msg.linear.z = 0;
        RCLCPP_INFO(this->get_logger(), "Publishing: %f , %f", this->msg.linear.x, this->msg.angular.z);
        publisher_->publish(msg);
    }

private:
    // Private variables used for the publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist msg;
};
*/

int main(int argc, char **argv)
{
    PaStream *stream;
    PaError err;
    DTMFDecoder decoder(1600);
    int result;
/*
    rclcpp::init(argc, argv);
    auto rb3_publisher = std::make_shared<RB3_cpp_publisher>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(rb3_publisher);
*/
    // Initialize PortAudio
    err = Pa_Initialize();

    PaStreamParameters inputParameters;
    inputParameters.device = Pa_GetDefaultInputDevice();
    inputParameters.channelCount = NUM_CHANNELS;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultHighInputLatency;
    inputParameters.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(&stream, &inputParameters, nullptr, SAMPLE_RATE, FRAMES_PER_BUFFER, paClipOff, nullptr, nullptr);

    err = Pa_StartStream(stream);
	
    std::cout << Pa_GetDeviceInfo(0) << std::endl;

    while (!shutdown)
    {

        std::vector<double> audioData;
        audioData.reserve(SAMPLE_RATE * NUM_CHANNELS * RECORDING_DURATION_SECONDS);

        for (int i = 0; i < RECORDING_DURATION_SECONDS * SAMPLE_RATE / FRAMES_PER_BUFFER; i++)
        {
            float buffer[FRAMES_PER_BUFFER];
            err = Pa_ReadStream(stream, buffer, FRAMES_PER_BUFFER);

            audioData.insert(audioData.end(), buffer, buffer + FRAMES_PER_BUFFER);
        }

        result = decoder.FFT(audioData, SAMPLE_RATE);
        switch (result)
        {
        case 1907:
            std::cout << "1" << std::endl;
            // rb3_publisher->publish_vel(0.1, 0);
            break;
        case 2033:
            std::cout << "2" << std::endl;
            // rb3_publisher->publish_vel(0, 0);

            break;
        case 2174:
            std::cout << "3" << std::endl;
            // rb3_publisher->publish_vel(0, 0.2);
            break;
        case 2330:
            std::cout << "A" << std::endl;
            break;
        case 1979:
            std::cout << "4" << std::endl;
            break;
        case 2106:
            std::cout << "5" << std::endl;
            break;
        case 2247:
            std::cout << "6" << std::endl;
            break;
        case 2403:
            std::cout << "B" << std::endl;
            break;
        case 2061:
            std::cout << "7" << std::endl;
            break;
        case 2188:
            std::cout << "8" << std::endl;
            break;
        case 2329:
            std::cout << "9" << std::endl;
            break;
        case 2485:
            std::cout << "C" << std::endl;
            break;
        case 2150:
            std::cout << "*" << std::endl;
            break;
        case 2277:
            std::cout << "0" << std::endl;
            break;
        case 2418:
            std::cout << "#" << std::endl;
            break;
        case 2574:
            std::cout << "D" << std::endl;
            shutdown = true;
            // rclcpp::shutdown();
            break;
        default:
            //std::cout << "Ewww" << std::endl;
            break;
        }
    }

    err = Pa_StopStream(stream);

    err = Pa_CloseStream(stream);

    Pa_Terminate();
    return 0;
}
