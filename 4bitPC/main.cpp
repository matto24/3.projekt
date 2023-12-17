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
#include <chrono>

#include "playAudio.h"
#include "FFT.h"
#include "PortAudioClass.h"
#include "RouteUI.h"
#include "CommandGenerator.h"

volatile char selectedKey = '\0';
volatile bool keepPlaying = false;

std::vector<float> recordBuffer;
bool shutdown = false;
const int sampleRate = 44100;
const int framesPrBuffer = 925;
const int numChannels = 1;

int main(int argc, char const *argv[])
{
    RouteUI ui;
    std::vector<std::string> moves = ui.run();
    PortAudioClass pa;
    pa.Initialize();
    PlayAudio audioPlayer;

    for (int m = 0; m < moves.size();)
    {
        int toneDuration = 20;
        int waitDuration = 20;
        int outputBuffer = sampleRate * (toneDuration + waitDuration) / 1000;

        std::string conversion = audioPlayer.toneList(moves[m]);
        std::cout << "Der afspilles: " << conversion << std::endl;
        char lastKey = '\0';
        int lastKeyCount = 0;
        pa.OpenOutputStream(sampleRate, outputBuffer, 1); // Open for playing
        pa.StartStream();

        for (char key : conversion)
        {
            // std::cout << key << std::endl;
            if (key == lastKey)
            {
                lastKeyCount++;
                if (lastKeyCount == 2)
                {
                    pa.PlayTone('0', toneDuration, waitDuration);
                }
                else
                {
                    pa.PlayTone(key, toneDuration, waitDuration);
                }
            }
            else
            {
                lastKey = key;
                lastKeyCount = 1;

                pa.PlayTone(key, toneDuration, waitDuration);
            }

            if (lastKeyCount == 2)
            {
                lastKey = key;
            }
        }

        pa.StopStream();
        // Venter på at lyd tråden er færdig med at spille
        // Optag nu

        int result;
        DTMFDecoder decoder(framesPrBuffer);

        auto start = std::chrono::high_resolution_clock::now();
        pa.OpenInputStream(sampleRate, framesPrBuffer, numChannels);
        pa.StartStream();

        while (!shutdown)
        {
            std::vector<float> buffer;
            pa.ReadStream(buffer, framesPrBuffer);
            result = decoder.FFT(buffer, sampleRate);
            if (result != 0)
            {
                std::cout << "result: " << result << std::endl;
            }
            // Tone 1
            if (result == 1906)
            {
                std::cout << "Play Next" << std::endl;
                m++;
                shutdown = true;
                std::this_thread::sleep_for(std::chrono::milliseconds(20)); // wait for ROBO to be ready
            }
            if (std::chrono::high_resolution_clock::now() - start > std::chrono::milliseconds(100) && !shutdown)
            {
                std::cout << "Play again" << std::endl;
                start = std::chrono::high_resolution_clock::now();
                shutdown = true;
            }
        }
        shutdown = false;
        pa.StopStream();
    }

    return 0;
}
