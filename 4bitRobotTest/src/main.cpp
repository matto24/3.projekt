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

#define OUTPUT_FILE "output.csv"

const int sampleRate = 44100;
const double recordingDurationSeconds = 0.05; // resolution = (sample_rate /(sample_rate*duration))
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

    // Testing
    int expiredCount = 0;
    int ackCount = 0;
    int checksumFailCount = 0;

    auto start = std::chrono::high_resolution_clock::now();

    while (!shutdown)
    {
        if(expiredCount + ackCount + checksumFailCount > 49){
            shutdown = true;
            std::cout << "ACK Count: " << ackCount << std::endl;
            std::cout << "Expired Count: " << expiredCount << std::endl;
            std::cout << "Checksum Fail Count: " << checksumFailCount << std::endl;
        }


        if (fundneToner.size() > 5)
        {
            decoder.setStartBit(false);
            decoder.clearLastSound();
            correctMessage = mi.interpretMessage(fundneToner);
            fundneToner.clear();

            if (correctMessage)
            {
                pa.StopStream();
                start = std::chrono::high_resolution_clock::now();
                // Venter 500ms før vi sender en ack
                usleep(500000);

                pa.OpenOutputStream(44100, 8820, 1); // Open for playing
                pa.StartStream();

                // Play the acknowledgment tone (example: 697 Hz and 1209 Hz for 1 second)
                pa.PlayTone(697, 1209, 100, 100);

                pa.StopStream();

                ackCount++;
            } else {
                checksumFailCount++;
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
        else if (std::chrono::high_resolution_clock::now() - start > std::chrono::seconds(2) && decoder.getStartBit())
        {
            start = std::chrono::high_resolution_clock::now();
            decoder.setStartBit(false);
            fundneToner.clear();
            decoder.clearLastSound();
            std::cout << "Timer expired -> Cleared fundne Toner" << std::endl;
            expiredCount++;
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
