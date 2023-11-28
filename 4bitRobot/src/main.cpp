#include <iostream>
#include <vector>
#include <map>
#include <utility>

#include "MessageInterpret.h"
#include "FFT.h"
#include "PortAudioClass.h"
#include "playAudio.h"

// #include "rb3_cpp_publisher.h"
#include "drive.h"
#include <unistd.h>

const int sampleRate = 32000;
const double recordingDurationSeconds = 0.05; // resolution = (sample_rate /(sample_rate*duration))
const int framesPrBuffer = 1600;
const int numChannels = 1;

volatile char selectedKey = '\0';
volatile bool keepPlaying = false;

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
    pa.OpenStream(sampleRate, framesPrBuffer, numChannels);
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
            correctMessage = mi.interpretMessage(fundneToner);
            fundneToner.clear();

            if (correctMessage)
            {
                start = std::chrono::high_resolution_clock::now();
                // Venter 500ms før vi sender en ack
                usleep(500000);
                Pa_Initialize();
                PaStream *playStream;
                Pa_OpenDefaultStream(&playStream, 0, 1, paFloat32, 44100, 4096, NULL, NULL);
                Pa_StartStream(playStream);
                pthread_t audioThreadId;
                // Instans a PlayAudio klassen
                PlayAudio audioPlayer;
                // Instans a struct der holder threadArgs.
                ThreadArgs threadArgs;
                threadArgs.stream = playStream;
                threadArgs.selectedKey = &selectedKey;
                threadArgs.keepPlaying = &keepPlaying;

                pthread_create(&audioThreadId, NULL, &PlayAudio::audioThread, (void *)&threadArgs);
                std::string acknowledgement = "1";
                std::cout << "Der afspilles ack" << std::endl;

                for (char key : acknowledgement)
                {
                    usleep(500000);
                    selectedKey = key;
                    keepPlaying = true;
                    usleep(1000000);
                }
                keepPlaying = false; // Ensure playback stops on exit
                ThreadArgs().stop = true;
                Pa_StopStream(playStream);
                Pa_CloseStream(playStream);
                Pa_Terminate();
            }

            if (mi.getExecuteRoute())
            {
                shutdown = true;
                // robo.commands(mi.getDriveComm62ands());
            }
        }
        else{
            if(std::chrono::high_resolution_clock::now()-start > std::chrono::seconds(2) && decoder.getStartBit()){
                start = std::chrono::high_resolution_clock::now();
                decoder.setStartBit(false);
                fundneToner.clear();
                std::cout << "Timer expired -> Cleared fundne Toner" << std::endl;
            }
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
            fundneToner.push_back(result);
            std::chrono::high_resolution_clock::now();


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