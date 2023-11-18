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
    std::string conversion = audioPlayer.toneList("101010100");
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

    std::cout << "Start listning" << std::endl;

    // Venter på at lyd tråden er færdig med at spille
    pthread_join(audioThreadId, NULL);
    std::cout << "1" << std::endl;
    Pa_StopStream(playStream);
    std::cout << "2" << std::endl;
    Pa_CloseStream(playStream);
    std::cout << "3" << std::endl;
    Pa_Terminate();
    std::cout << "4" << std::endl;

    //Optag nu


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
            //Tone C
            if (result == 1907)
            {
                shutdown = true;
            }
        }
    }
    std::cout << "Tak for i dag" << std::endl;

    return 0;
}