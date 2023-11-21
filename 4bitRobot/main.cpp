#include <iostream>
#include <vector>
#include <map>
#include <utility>

#include "MessageInterpret.h"
#include "FFT.h"
#include "PortAudioClass.h"
// #include "rb3_cpp_publisher.h"
// #include "drive.h"
#include <unistd.h>

const int sampleRate = 8000;
const double recordingDurationSeconds = 0.2; // resolution = (sample_rate /(sample_rate*duration))
const int framesPrBuffer = 1600;
const int numChannels = 1;

int main(int argc, char **argv)
{
    /*
    rclcpp::init(argc, argv);
    auto rb3_publisher = std::make_shared<RB3_cpp_publisher>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(rb3_publisher);

    Drive robo(rb3_publisher); */

    DTMFDecoder decoder(1600);
    MessageInterpreter mi;

    PortAudioClass pa;
    pa.Initialize();
    pa.OpenStream(sampleRate, framesPrBuffer, numChannels);
    pa.StartStream();

    int result;
    std::vector<int> fundneToner;

    bool startBit = false;
    bool shutdown = false;

    while (!shutdown)
    {
        if(fundneToner.size() > 5){
            startBit = false;
            mi.interpretMessage(fundneToner);
            fundneToner.clear();
            if(mi.getExecuteRoute()){
                shutdown = false;
                //robo.commands(mi.getDrieCommands);
            }
        }

        std::vector<float> buffer;
        pa.ReadStream(buffer, framesPrBuffer);
        result = decoder.FFT(buffer, sampleRate);

        if (result != 0)
        {
            std::cout << result << std::endl;
        }

        if (result == 2277 && !startBit) // tonen 0 og startbit = false
        {
            startBit = true;
            fundneToner.clear();
            std::cout << "start" << std::endl;

            continue;
        }

        if (startBit && result != 0)
        {
            fundneToner.push_back(result);
            continue;
        }
    }
    // rclcpp::shutdown();
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