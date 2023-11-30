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
// #include "playAudio.h"

// #include "rb3_cpp_publisher.h"
#include "drive.h"

#define OUTPUT_FILE "output.csv"

const int sampleRate = 32000;
const double recordingDurationSeconds = 0.05; // resolution = (sample_rate /(sample_rate*duration))
const int framesPrBuffer = 1200;
const int numChannels = 1;

int main(int argc, char **argv)
{
    // rclcpp::init(argc, argv);
    // auto rb3_publisher = std::make_shared<RB3_cpp_publisher>();
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(rb3_publisher);
    // Drive robo(rb3_publisher);

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
                start = std::chrono::high_resolution_clock::now();
                // Venter 500ms før vi sender en ack
                usleep(500000);

                pa.OpenOutputStream(44100, 4096, 1); // Open for playing
                pa.StartStream();

                // Play the acknowledgment tone (example: 697 Hz and 1209 Hz for 1 second)
                pa.PlayTone(697, 1209, 1.0);

                pa.StopStream();
            }

            if (mi.getExecuteRoute())
            {
                shutdown = true;
                // robo.commands(mi.getDriveComm62ands());
            }
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
        }

        std::vector<float> buffer;
        pa.ReadStream(buffer, framesPrBuffer);
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
            // std::ofstream outputFile(OUTPUT_FILE);
            // for (size_t i = 0; i < buffer.size(); i++)
            // {
            //     outputFile << buffer[i];
            //     if (i < buffer.size() - 1)
            //     {
            //         outputFile << ",";
            //     }
            // }
            // outputFile.close();

            fundneToner.push_back(result);
            start = std::chrono::high_resolution_clock::now();

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
