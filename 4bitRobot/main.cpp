#include <iostream>
#include <vector>
#include <map>
#include <utility>

#include "MessageInterpret.h"
#include "FFT.h"
#include "PortAudioClass.h"
//#include "rb3_cpp_publisher.h"
//#include "drive.h"
#include <unistd.h>

const int sampleRate = 8000;
const double recordingDurationSeconds = 0.2; // resolution = (sample_rate /(sample_rate*duration))
const int framesPrBuffer = 1600;
const int numChannels = 1;

bool shutdown = false;



int main(int argc, char **argv)
{
    /*
    rclcpp::init(argc, argv);
    auto rb3_publisher = std::make_shared<RB3_cpp_publisher>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(rb3_publisher);

    Drive robo(rb3_publisher); */

    int result;
    DTMFDecoder decoder(1600);

    PortAudioClass pa;
    pa.Initialize();
    pa.OpenStream(sampleRate, framesPrBuffer, numChannels);
    pa.StartStream();

    const size_t ringBufferSize = sampleRate * numChannels * recordingDurationSeconds;
    std::vector<double> ringBuffer(ringBufferSize, 0.0);
    size_t ringBufferIndex = 0;
    std::vector<int> fundneToner;
    bool startBit = false;

    while (!shutdown)
    {
        std::vector<float> buffer;
        pa.ReadStream(buffer, framesPrBuffer);

        // Copy into ring buffer
        for (int i = 0; i < framesPrBuffer; ++i)
        {
            ringBuffer[ringBufferIndex] = buffer[i];
            ringBufferIndex = (ringBufferIndex + 1) % ringBufferSize;
        }
        // If the ring buffer is filled, process it
        if (ringBufferIndex == 0)
        {   
            result = decoder.FFT(ringBuffer, sampleRate);
            if (result != 0)
            {
                std::cout << result << std::endl;
            }

            if (result == 2150 && !startBit)
            {
                startBit = true;
                fundneToner.clear();
                std::cout << "start" << std::endl;

                continue;
            }
            else if (result == 2418 && startBit)
            { // "# - stopbit"
                std::cout << "end" << std::endl;
                interpretMessage(fundneToner, &robo);
                startBit = false;

                continue;
            }

            if (startBit && result != 0)
            {
                fundneToner.push_back(result);
            }
            if(result == 2574) {
                robo.commands(driveCommands);
                shutdown = true;
            }
        }
    }
    rclcpp::shutdown();
    return 0;
}

/*
1907: 1
2033: 2
2174: 3
2330: A
1979: 4
2106: 5
2247: 6
2403: B
2061: 7
2188: 8
2329: 9
2485: C
2150: *
2277: 0
2418: #
2574: D

*/