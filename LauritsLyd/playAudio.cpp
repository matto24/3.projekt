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
#include <thread>
#include <mutex>

#include "playAudio.h"

const double PlayAudio::dtmfFrequencies[4][4] = {
    {697, 770, 852, 941},
    {1209, 1336, 1477, 1633}};

void PlayAudio::generateDTMFTone(char key, float *buffer, int frames)
{
    int row, col;

    switch (key)
    {
    case '1':
        row = 0;
        col = 0;
        break;
    case '2':
        row = 0;
        col = 1;
        break;
    case '3':
        row = 0;
        col = 2;
        break;
    case 'A':
        row = 0;
        col = 3;
        break;

    case '4':
        row = 1;
        col = 0;
        break;
    case '5':
        row = 1;
        col = 1;
        break;
    case '6':
        row = 1;
        col = 2;
        break;
    case 'B':
        row = 1;
        col = 3;
        break;

    case '7':
        row = 2;
        col = 0;
        break;
    case '8':
        row = 2;
        col = 1;
        break;
    case '9':
        row = 2;
        col = 2;
        break;
    case 'C':
        row = 2;
        col = 3;
        break;

    case '*':
        row = 3;
        col = 0;
        break;
    case '0':
        row = 3;
        col = 1;
        break;
    case '#':
        row = 3;
        col = 2;
        break;
    case 'D':
        row = 3;
        col = 3;
        break;

    default:
        row = -1;
        col = -1;
        break;
    }

    if (row != -1 && col != -1)
    {
        double freq1 = dtmfFrequencies[0][row];
        double freq2 = dtmfFrequencies[1][col];

        for (int i = 0; i < frames; i++)
        {
            double t = (double)i / SAMPLE_RATE;
            buffer[i] = AMPLITUDE * (sin(2 * PI * freq1 * t) + sin(2 * PI * freq2 * t));
        }
    }
    else
    {
        for (int i = 0; i < frames; i++)
        {
            buffer[i] = 0.0; // If invalid key, generate silence
        }
    }
}

std::mutex mu;

void *PlayAudio::audioThread(void *args)
{
    mu.lock();
    ThreadArgs *threadArgs = (ThreadArgs *)args;
    PaStream *stream = threadArgs->stream;
    volatile char *selectedKey = threadArgs->selectedKey;
    volatile bool *keepPlaying = threadArgs->keepPlaying;

    int frames = (int)(SAMPLE_RATE * TONE_DURATION); // Adjust buffer size

    float buffer[frames]; // Fixed-size buffer

    while (!threadArgs->stop)
    {
        if (*keepPlaying)
        {
            generateDTMFTone(*selectedKey, buffer, frames);
            Pa_WriteStream(stream, buffer, frames);

            *keepPlaying = false;
            *selectedKey = '\0'; // Reset selected key after playing
        }
    }

    std::cout << std::this_thread::get_id() << ": Lort" << std::endl;

    pthread_exit(NULL);

    mu.unlock();
    return NULL;
}

std::string PlayAudio::toneList(std::string binaryNum)
{
    std::map<std::string, std::string> toneToBitMap = {{"0", "000"}, {"1", "001"}, {"2", "010"}, {"3", "011"}, {"4", "100"}, {"5", "101"}, {"6", "110"}, {"7", "111"}, {"8", "0"}, {"9", "1"}};
    std::string output;

    // Check length of binary number. If binaryNum%3 != 0, then append 0s at beginning to make it.
    while (binaryNum.size() % 3 != 0)
    {
        binaryNum.insert(12, "0");
    }
    // Iterator
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

    return "*" +output +"#";
}