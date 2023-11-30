#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <unistd.h>
#include <map>
#include <vector>
#include <iostream>
#include <bits/stdc++.h>
#include <thread>
#include <mutex>

#include "playAudio.h"
#include "FFT.h"
#include "PortAudioClass.h"
#include "RouteUI.h"
#include "CommandGenerator.h"

#define SAMPLE_RATE 44100

volatile char selectedKey = '\0';
volatile bool keepPlaying = false;

std::vector<float> recordBuffer;
bool shutdown = false;
const int sampleRate = 8000;
const int framesPrBuffer = 1600;
const double recordingDurationSeconds = 0.2;
const int numChannels = 1;

int main(int argc, char const *argv[])
{

    RouteUI ui;
    std::vector<std::string> moves = ui.run();

    for (int m = 0; m < moves.size();)
    {
        Pa_Initialize();
        PaStream *playStream;

        Pa_OpenDefaultStream(&playStream, 0, 1, paFloat32, SAMPLE_RATE, 4096, NULL, NULL);
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

        std::string conversion = audioPlayer.toneList(moves[m]);
        std::cout << "Der afspilles: " << conversion << std::endl;
        char lastKey = '\0';
        int lastKeyCount = 0;

        for (char key : conversion)
        {
            if (key == lastKey)
            {
                lastKeyCount++;
                if (lastKeyCount == 2)
                {
                    selectedKey = '0';
                }
                else
                {
                    selectedKey = key;
                }
            }
            else
            {
                lastKey = key;
                lastKeyCount = 1;
                selectedKey = key;
            }
            auto startTimeTest = std::chrono::high_resolution_clock::now();
            keepPlaying = true;
            usleep(140000); // tidligere usleep(1000000);
            auto CurrentTimeTest = std::chrono::high_resolution_clock::now();
            auto elapsedTimeTest = std::chrono::duration_cast<std::chrono::milliseconds>(CurrentTimeTest - startTimeTest).count();
            std::cout << "tid om at fylde buffer: " << elapsedTimeTest << std::endl;
            if (lastKeyCount == 2)
            {
                lastKey = key;
            }
        }

        keepPlaying = false; // Ensure playback stops on exit

        ThreadArgs().stop = true;

        // Venter på at lyd tråden er færdig med at spille

        Pa_StopStream(playStream);
        Pa_CloseStream(playStream);
        Pa_Terminate();

        // Optag nu

        int result;
        DTMFDecoder decoder(1600);

        PortAudioClass pa;
        pa.Initialize();
        pa.OpenStream(8000, framesPrBuffer, numChannels);
        // Start recording stream
        pa.StartStream();

        const size_t ringBufferSize = sampleRate * recordingDurationSeconds;
        std::vector<double> ringBuffer(ringBufferSize, 0.0);
        size_t ringBufferIndex = 0;
        std::vector<int> fundneToner;

        auto start = std::chrono::high_resolution_clock::now();

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
                    std::cout << "result " << result << std::endl;
                }
                // Tone 1
                if (result == 1907)
                {
                    shutdown = true;
                    std::cout << "Play Next" << std::endl;
                    m++;
                    usleep(1000000);
                }
            }
            if (std::chrono::high_resolution_clock::now() - start > std::chrono::seconds(3))
            {
                std::cout << "Play again" << std::endl;
                shutdown = true;
                usleep(1000000);
            }
        }
        shutdown = false;
        pa.StopStream();
    }

    return 0;
}