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
const int sampleRate = 44100;
const int framesPrBuffer = 925;
const double recordingDurationSeconds = 0.2;
const int numChannels = 1;

int main(int argc, char const *argv[]) {
  RouteUI ui;
  // std::vector<std::string> moves = ui.run();
  std::vector<std::string> moves;
  for (int i = 0; i < 6; i++) {

    // 7,6,3,9,D,A
    moves.push_back("0000011101100011100111011010");
    // 7, 2, 5,
    moves.push_back("0000011100100101100011011100");
    // 9, 1, 6,
    moves.push_back("0000100100010110100010111111");
    // D, 3, *, C, 8, 0
    moves.push_back("0000110100111110100110011011");
    // 7, 9, 4, A, B, 0
    moves.push_back("0000011110010100101010110000");
    moves.push_back("0000110100111110100110011011");
  }

  PortAudioClass pa;
  pa.Initialize();
  PlayAudio audioPlayer;
  int replay = 0;
  std::string allTones;
  auto startTimeTest = std::chrono::high_resolution_clock::now();
  for (int m = 0; m < moves.size();) {
    std::cout << "Move: " << m << std::endl;
    int toneDuration = 15;
    int waitDuration = 20;
    int outputBuffer = 44100 * (toneDuration + waitDuration) / 1000;

    std::string conversion = audioPlayer.toneList(moves[m]);
    std::cout << "Der afspilles: " << conversion << std::endl;
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
    DTMFDecoder decoder(framesPrBuffer);
    auto start = std::chrono::high_resolution_clock::now();
    pa.OpenInputStream(sampleRate, framesPrBuffer, numChannels);
    pa.StartStream();

    while (!shutdown) {

      std::vector<float> buffer;
      pa.ReadStream(buffer, framesPrBuffer);

      // If the ring buffer is filled, process it
        result = decoder.FFT(buffer, sampleRate);
        if (result != 0) {
          std::cout << "result " << result << std::endl;
        }
        // Tone 1
        if (result == 1906) {
          shutdown = true;
          std::cout << "Play Next" << std::endl;
          m++;
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      
      if (std::chrono::high_resolution_clock::now() - start >
              std::chrono::milliseconds(100) &&
          !shutdown) {
        std::cout << "Play again" << std::endl;
        shutdown = true;
        m++;
      }
    }
    shutdown = false;
    pa.StopStream();
  }
  // cout how many times each character is used in allTones
  auto CurrentTimeTest = std::chrono::high_resolution_clock::now();
  auto elapsedTimeTest = std::chrono::duration_cast<std::chrono::milliseconds>(
                             CurrentTimeTest - startTimeTest)
                             .count();
  std::cout << "Tid om afspilning: " << elapsedTimeTest << std::endl;
  return 0;
}
