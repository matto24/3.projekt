#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <unistd.h>
#include <map>
#include <vector>
#include <iostream>
#include <bits/stdc++.h>

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
int framesPrBuffer = 1600;
double recordingDurationSeconds = 0.2;
const int numChannels = 1;

int main(int argc, char const *argv[])
{

    RouteUI ui;
    std::vector<std::string> moves = ui.run();

    for (int i = 0; i< moves.size();)
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

        std::string conversion = audioPlayer.toneList(moves[i]);
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
                    selectedKey = 'A';
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

            keepPlaying = true;
            usleep(1000000);

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
        //pthread_join(audioThreadId, NULL);

        // Optag nu

        int result;
        DTMFDecoder decoder(1600);

        PortAudioClass pa;
        pa.Initialize();
        pa.OpenStream(8000, framesPrBuffer, numChannels);
        // Start recording stream
        pa.StartStream();

        const size_t ringBufferSize = 8000 * recordingDurationSeconds;
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
                result = decoder.FFT(ringBuffer, SAMPLE_RATE);
                std::cout << "result " << result << std::endl;
                // Tone C
                if (result == 1907)
                {
                    shutdown = true;
                    std::cout << "Play Next" << std::endl;
                    i++;
                }
            }
            if(std::chrono::high_resolution_clock::now()-start > std::chrono::seconds(5)){
                std::cout << "Play again" << std::endl;
                shutdown = true;
            }
        }
        shutdown = false;
        pa.StopStream();
        
    }

    return 0;
}