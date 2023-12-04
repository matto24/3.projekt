#include <iostream>
#include <vector>
#include <map>
#include <utility>
#include <unistd.h> //usleep
#include <chrono>

#include "MessageInterpret.h"
#include "FFT.h"
#include "PortAudioClass.h"
// #include "playAudio.h"

// #include "rb3_cpp_publisher.h"
#include "drive.h"

const int sampleRate = 44000;
const double recordingDurationSeconds = 0.05; // resolution = (sample_rate /(sample_rate*duration))
const int framesPrBuffer = 1850;
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

    char result;
    std::vector<char> fundneToner;

    decoder.setStartBit(false);
    bool shutdown = false;
    bool correctMessage = false;

    auto start = std::chrono::high_resolution_clock::now();

    while (!shutdown)
    {
        auto test1 = std::chrono::high_resolution_clock::now();
        if (fundneToner.size() > 5)
        {
            for (auto pik : fundneToner){
                std::cout << pik;
            }
            std::cout << std::endl;
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
            std::cout << "count" << fundneToner.size() << std::endl;
            fundneToner.clear();
            decoder.clearLastSound();
            std::cout << "Timer expired -> Cleared fundne Toner" << std::endl;
        }

        std::vector<float> buffer;
        pa.ReadStream(buffer, framesPrBuffer);
        result = decoder.FFT(buffer, sampleRate);

        if (decoder.getStartBit() && result != 'x')
        {
            fundneToner.push_back(result);
            usleep(50000);
            start = std::chrono::high_resolution_clock::now();

            //continue;
        }
        if (result == '0' && !decoder.getStartBit()) // tonen 0 og startbit = false
        {
            decoder.setStartBit(true);
            fundneToner.clear();
            start = std::chrono::high_resolution_clock::now();
            std::cout << "Start" << std::endl;
            //continue;
        }
        if (result != 'x' && result != '0')
        {
            std::cout << result << std::endl;
        }

        auto test2 = std::chrono::high_resolution_clock::now();
        //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(test2 - test1).count() << std::endl;

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
