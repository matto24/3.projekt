#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <portaudio.h>
#include <cmath>
#include <chrono>
#include "class2.hpp"
//#include "robot.hpp"


#include "portaudio.h"

#define SAMPLE_RATE 8000
#define RECORDING_DURATION_SECONDS 0.2 // resolution = (sample_rate /(sample_rate*duration))
#define FRAMES_PER_BUFFER 1600 // 1600
#define NUM_CHANNELS 1

using namespace std;

bool shutdown = false;

int main(int argc, char **argv)
{
    PaStream *stream;
    PaError err;
    DTMFDecoder decoder(1600);
    int result;
    
    /*rclcpp::init(argc, argv);
    auto rb3_publisher = std::make_shared<RB3_cpp_publisher>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(rb3_publisher); */
    
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

    const size_t ringBufferSize = SAMPLE_RATE * NUM_CHANNELS * RECORDING_DURATION_SECONDS;
    std::vector<double> ringBuffer(ringBufferSize, 0.0);
    size_t ringBufferIndex = 0;

    while (!shutdown)
    {

        float buffer[FRAMES_PER_BUFFER];
        err = Pa_ReadStream(stream, buffer, FRAMES_PER_BUFFER);

        auto start = std::chrono::high_resolution_clock::now();
        

        // Copy into ring buffer
        for (int i = 0; i < FRAMES_PER_BUFFER; ++i)
        {
            ringBuffer[ringBufferIndex] = buffer[i];
            ringBufferIndex = (ringBufferIndex + 1) % ringBufferSize;
        }

        // If the ring buffer is filled, process it
        if (ringBufferIndex == 0)
        {
            result = decoder.FFT(ringBuffer, SAMPLE_RATE);
            // ... switch case for DTMF tones ...
            
            
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
                // std::cout << "Ewww" << std::endl;
                break;
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
        std::cout << "Processing cycle took " << duration.count() << " milliseconds.\n";
    }

    err = Pa_StopStream(stream);

    err = Pa_CloseStream(stream);

    Pa_Terminate();
    return 0;
}
