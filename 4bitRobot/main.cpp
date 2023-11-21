#include <iostream>
#include <vector>
#include <map>
#include <utility>

#include "MessageInterpret.h"
#include "FFT.h"
#include "PortAudioClass.h"
#include "playAudio.h"

// #include "rb3_cpp_publisher.h"
// #include "drive.h"
#include <unistd.h>

const int sampleRate = 8000;
const double recordingDurationSeconds = 0.2; // resolution = (sample_rate /(sample_rate*duration))
const int framesPrBuffer = 1600;
const int numChannels = 1;

volatile char selectedKey = '\0';
volatile bool keepPlaying = false;

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
        if (fundneToner.size() > 5)
        {
            startBit = false;
            mi.interpretMessage(fundneToner);
            fundneToner.clear();
            if (mi.getExecuteRoute())
            {
                shutdown = true;
                // robo.commands(mi.getDriveCommands);
            }
            
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