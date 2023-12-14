#include <bits/stdc++.h>
#include <chrono>
#include <iostream>
#include <map>
#include <math.h>
#include <mutex>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "CommandGenerator.h"
#include "FFT.h"
#include "PortAudioClass.h"
#include "RouteUI.h"
#include "playAudio.h"

#define SAMPLE_RATE 44100

volatile char selectedKey = '\0';
volatile bool keepPlaying = false;

std::vector<float> recordBuffer;
bool shutdown = false;
const int sampleRate = 8000;
const int framesPrBuffer = 1600;
const double recordingDurationSeconds = 0.2;
const int numChannels = 1;

int main(int argc, char const *argv[]) {
  RouteUI ui;
  // std::vector<std::string> moves = ui.run();
  std::vector<std::string> moves;
    // Kør en meter
    moves.push_back("0000110111111010110010001111");
    // Execute
    moves.push_back("0000001100110100100011101111");

  PortAudioClass pa;
  pa.Initialize();
  PlayAudio audioPlayer;
  int ackCount = 0;
  std::string allTones;
  auto startTimeTest = std::chrono::high_resolution_clock::now();
  for (int m = 0; m < moves.size();) {
    int toneDuration = 20;
    int waitDuration = 20;
    int outputBuffer = 44100 * (toneDuration + waitDuration) / 1000;

    std::string conversion = audioPlayer.toneList(moves[m]);
    std::cout << "Der afspilles: " << conversion << std::endl;
    std::cout << "Command nr: " << m << std::endl;
    allTones += conversion;
    char lastKey = '\0';
    int lastKeyCount = 0;
    pa.OpenOutputStream(44100, outputBuffer, 1); // Open for playing
    pa.StartStream();

    for (char key : conversion) {
      auto test = std::chrono::high_resolution_clock::now();
      // std::cout << key << std::endl;
      if (key == lastKey) {
        lastKeyCount++;
        if (lastKeyCount == 2) {
          pa.PlayTone('0', toneDuration, waitDuration);
        } else {
          pa.PlayTone(key, toneDuration, waitDuration);
        }
        // usleep(waitDuration*1000);
      } else {
        lastKey = key;
        lastKeyCount = 1;

        // Play the acknowledgment tone (example: 697 Hz and 1209 Hz for 1
        // second)
        pa.PlayTone(key, toneDuration, waitDuration);
        // usleep(waitDuration*1000);
      }
      std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::high_resolution_clock::now() - test)
                       .count()
                << std::endl;

      if (lastKeyCount == 2) {
        lastKey = key;
      }
    }

    pa.StopStream();

    // Venter på at lyd tråden er færdig med at spille

    // Optag nu

    int result;
    DTMFDecoder decoder(1600);

    const size_t ringBufferSize = sampleRate * recordingDurationSeconds;
    std::vector<double> ringBuffer(ringBufferSize, 0.0);
    size_t ringBufferIndex = 0;
    std::vector<int> fundneToner;

    auto start = std::chrono::high_resolution_clock::now();
    pa.OpenInputStream(sampleRate, framesPrBuffer, numChannels);
    pa.StartStream();

    while (!shutdown) {

      std::vector<float> buffer;
      pa.ReadStream(buffer, framesPrBuffer);
      // Copy into ring buffer
      for (int i = 0; i < framesPrBuffer; ++i) {
        ringBuffer[ringBufferIndex] = buffer[i];
        ringBufferIndex = (ringBufferIndex + 1) % ringBufferSize;
      }
      // If the ring buffer is filled, process it
      if (ringBufferIndex == 0) {
        result = decoder.FFT(ringBuffer, sampleRate);
        if (result != 0) {
          std::cout << "result " << result << std::endl;
        }
        // Tone 1
        if (result == 1907) {
          shutdown = true;
          std::cout << "Play Next" << std::endl;
          m++;
          ackCount++;
          std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
      }
      if (std::chrono::high_resolution_clock::now() - start >
              std::chrono::milliseconds(500) &&
          !shutdown) {
        std::cout << "Play again" << std::endl;
        shutdown = true;
        // usleep(500000);
      }
    }
    shutdown = false;
    pa.StopStream();
  }
  std::cout << "antal ack: " << ackCount << std::endl;
  // cout how many times each character is used in allTones
  
  std::map<char, int> charCount;
  for (char c : allTones) {
    charCount[c]++;
  }
  for (auto it = charCount.begin(); it != charCount.end(); it++) {
    std::cout << it->first << ": " << it->second << std::endl;
  }
  auto CurrentTimeTest = std::chrono::high_resolution_clock::now();
  auto elapsedTimeTest = std::chrono::duration_cast<std::chrono::milliseconds>(CurrentTimeTest - startTimeTest).count();
  std::cout << "Tid om afspilning: " << elapsedTimeTest << std::endl;  
  return 0;
}
