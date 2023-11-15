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

// Define DTMF frequencies
#define SAMPLE_RATE 44100
#define PI 3.14159265358979323846
#define AMPLITUDE 0.5
#define TONE_DURATION 0.1

// DTMF Frequencies (Hz)
double dtmfFrequencies[4][4] = {
    {697, 770, 852, 941},
    {1209, 1336, 1477, 1633}
};

volatile char selectedKey = '\0';

// Function to generate a DTMF tone
void generateDTMFTone(char key, float* buffer, int frames) {
    int row, col;

    switch (key) {
        case '1': row = 0; col = 0; break;
        case '2': row = 0; col = 1; break;
        case '3': row = 0; col = 2; break;
        case 'A': row = 0; col = 3; break;

        case '4': row = 1; col = 0; break;
        case '5': row = 1; col = 1; break;
        case '6': row = 1; col = 2; break;
        case 'B': row = 1; col = 3; break;

        case '7': row = 2; col = 0; break;
        case '8': row = 2; col = 1; break;
        case '9': row = 2; col = 2; break;
        case 'C': row = 2; col = 3; break;

        case '*': row = 3; col = 0; break;
        case '0': row = 3; col = 1; break;
        case '#': row = 3; col = 2; break;
        case 'D': row = 3; col = 3; break;

        default: row = -1; col = -1; break;
    }

    if (row != -1 && col != -1) {
        double freq1 = dtmfFrequencies[0][row];
        double freq2 = dtmfFrequencies[1][col];
        
        for (int i = 0; i < frames; i++) {
            double t = (double)i / SAMPLE_RATE;
            buffer[i] = AMPLITUDE * (sin(2 * PI * freq1 * t) + sin(2 * PI * freq2 * t));
        }
    }
    else {
        for (int i = 0; i < frames; i++) {
            buffer[i] = 0.0; // If invalid key, generate silence
        }
    }
}

volatile bool keepPlaying = false;

void* audioThread(void* args) {
    PaStream* stream = (PaStream*)args;
    int frames = (int)(SAMPLE_RATE * TONE_DURATION); // Adjust buffer size

    float buffer[frames];  // Fixed-size buffer

    while (1) {
        if (keepPlaying) {
            generateDTMFTone(selectedKey, buffer, frames);
            Pa_WriteStream(stream, buffer, frames);

            keepPlaying = false;
            selectedKey = '\0';  // Reset selected key after playing
        }
    }

    return NULL;
}

std::string toneList(std::string binaryNum)
{
    std::map<std::string, std::string> toneToBitMap = {{"0", "000"}, {"1", "001"}, {"2", "010"}, {"3", "011"}, {"4", "100"}, {"5", "101"}, {"6", "110"}, {"7", "111"}, {"8", "0"}, {"9", "1"}};
    std::string output;

    // Check length of binary number. If binaryNum%3 != 0, then append 0s at beginning to make it.
    while (binaryNum.size() % 3 != 0)
    {
        binaryNum.insert(12, "0");
    }
    //Iterator
    std::map<std::string, std::string>::iterator itr;
    for (int i = 0; i < binaryNum.size() / 3; i++)
    {
        std::string firstThree = binaryNum.substr(3 * i, 3);
        for (itr = toneToBitMap.begin(); itr != toneToBitMap.end(); ++itr)
        {
            if (itr->second == firstThree)
            {
                output += itr->first;
                break;
            }
        }
    }

    return output;
}


int main() {
    Pa_Initialize();
    PaStream* stream;
    Pa_OpenDefaultStream(&stream, 0, 1, paFloat32, SAMPLE_RATE, 4096, NULL, NULL);
    Pa_StartStream(stream);

    pthread_t audioThreadId;
    pthread_create(&audioThreadId, NULL, audioThread, (void*)stream);    


    std::string conversion = toneList("010101000");

    char keys[conversion.length()+1]; // List of keys to play
    strcpy(keys, conversion.c_str());

    for (int i = 0; i < sizeof(keys) - 1; i++) {
        selectedKey = keys[i];
        keepPlaying = true;
        usleep(100000*10*1); // Sleep for 0.1 seconds (adjust as needed)
    }

    keepPlaying = false; // Ensure playback stops on exit

    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();

    pthread_join(audioThreadId, NULL);  // Wait for audio thread to finish

    return 0;
}