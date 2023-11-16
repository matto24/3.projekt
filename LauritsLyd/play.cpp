#include <stdio.h>
#include <portaudio.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <unistd.h>
#include <map>
#include <vector>
#include <iostream>
#include <bits/stdc++.h> 

#include "playAudio.h"

#define SAMPLE_RATE 44100

volatile char selectedKey = '\0';
volatile bool keepPlaying = false;

int main(int argc, char const *argv[]) {
    
    Pa_Initialize();
    PaStream* stream;
    Pa_OpenDefaultStream(&stream, 0, 1, paFloat32, SAMPLE_RATE, 4096, NULL, NULL);
    Pa_StartStream(stream);
    pthread_t audioThreadId;

    PlayAudio audioPlayer;
    ThreadArgs threadArgs;
    threadArgs.stream = stream;
    threadArgs.selectedKey = &selectedKey;
    threadArgs.keepPlaying = &keepPlaying;

    pthread_create(&audioThreadId, NULL, &PlayAudio::audioThread, (void*)&threadArgs);    
    std::string conversion = audioPlayer.toneList("0101010101010101010101010101");
    std::cout << "Der afspilles: " << conversion << std::endl;
    char lastKey = '\0';
    int lastKeyCount = 0;

    for (char key : conversion){
        if (key == lastKey){
            lastKeyCount++;
            if (lastKeyCount == 2) {
                selectedKey = 'A';
            } else {
                selectedKey = key;
            }
        } else {
            lastKey = key;
            lastKeyCount = 1;
            selectedKey = key;
        }

        keepPlaying = true;
        usleep(100000*10*1);

        if (lastKeyCount == 2) {
            lastKey = key;
        }
    }

    keepPlaying = false; // Ensure playback stops on exit

    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
    pthread_join(audioThreadId, NULL);  // Wait for audio thread to finish

    return 0;
}