#include <iostream>
#include <vector>
#include <map>
#include <utility>
#include <unistd.h> //usleep
#include <chrono>
#include <fstream>
#include "MessageInterpret.h"
#include "FFT.h"
#include "PortAudioClass.h"
#include "rb3_cpp_publisher.h"
#include "drive.h"

const int sampleRate = 44100; // resolution = (sample_rate /(sample_rate*duration))
const int framesPrBuffer = 925;
const int numChannels = 1;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto rb3_publisher = std::make_shared<RB3_cpp_publisher>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(rb3_publisher);
    Drive robo(rb3_publisher);

    DTMFDecoder decoder(framesPrBuffer);
    MessageInterpreter mi;

    PortAudioClass pa;
    pa.Initialize();
    pa.OpenInputStream(sampleRate, framesPrBuffer, numChannels);
    pa.StartStream();

    int result;
    std::vector<int> fundneToner;

    decoder.setStartBit(false);
    bool shutdown = false;
    bool correctMessage = false;

    auto start = std::chrono::high_resolution_clock::now();

    while (!shutdown)
    {
        if (fundneToner.size() > 5)
        {
            decoder.setStartBit(false);
            decoder.clearLastSound();
            correctMessage = mi.interpretMessage(fundneToner);
            fundneToner.clear();

            if (correctMessage)
            {
                std::cout << "Message is correct" << std::endl;
                pa.StopStream();
                start = std::chrono::high_resolution_clock::now();
                
                pa.OpenOutputStream(44100, 1764, 1); // Open for playing
                std::cout << "åbnet output stream" << std::endl;
                pa.StartStream();
                std::cout << "startet stream" << std::endl;

                // Play the acknowledgment
                pa.PlayTone(697, 1209, 20, 20);
                pa.StopStream();
            }

            if (mi.getExecuteRoute())
            {
                shutdown = true;
                robo.commands(mi.getDriveCommands());
            }
            pa.StopStream();
            pa.OpenInputStream(sampleRate, framesPrBuffer, numChannels);
            pa.StartStream();
        }
        else if (std::chrono::high_resolution_clock::now() - start > std::chrono::milliseconds(60) && decoder.getStartBit())
        {
            start = std::chrono::high_resolution_clock::now();
            decoder.setStartBit(false);
            fundneToner.clear();
            decoder.clearLastSound();
            std::cout << "Timer expired -> Cleared fundne Toner" << std::endl;
        }

        std::vector<float> buffer;
        //auto startTimeTest = std::chrono::high_resolution_clock::now();
        pa.ReadStream(buffer, framesPrBuffer);
        //auto CurrentTimeTest = std::chrono::high_resolution_clock::now();
        //auto elapsedTimeTest = std::chrono::duration_cast<std::chrono::milliseconds>(CurrentTimeTest - startTimeTest).count();
        //std::cout << "Tid om at fylde bufferen: " << elapsedTimeTest << std::endl;
        result = decoder.FFT(buffer, sampleRate);

        
        if (result != 0 && result != 2277)
        {
            std::cout << result << std::endl;
        }

        if (result == 2277 && !decoder.getStartBit()) // tonen 0 og startbit = false
        {
            decoder.setStartBit(true);
            fundneToner.clear();
            start = std::chrono::high_resolution_clock::now();
            std::cout << "Start" << std::endl;
            continue;
        }

        //Save tones
        else if (decoder.getStartBit() && result != 0)
        {
            fundneToner.push_back(result);
            start = std::chrono::high_resolution_clock::now();

            continue;
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
