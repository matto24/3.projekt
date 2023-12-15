#include "FFT.h"
#include "MessageInterpret.h"
#include "PortAudioClass.h"
#include <chrono>
#include <iostream>
#include <map>
#include <unistd.h> //usleep
#include <utility>
#include <vector>
// #include "rb3_cpp_publisher.h"
// #include "drive.h"

#define OUTPUT_FILE "output.csv"

const int sampleRate = 44100;
const double recordingDurationSeconds =
    0.05; // resolution = (sample_rate /(sample_rate*duration))
const int framesPrBuffer = 925;
const int numChannels = 1;

int main(int argc, char **argv) {

  // rclcpp::init(argc, argv);
  // auto rb3_publisher = std::make_shared<RB3_cpp_publisher>();
  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(rb3_publisher);
  // Drive robo(rb3_publisher);

  DTMFDecoder decoder(framesPrBuffer);
  MessageInterpreter mi;

  PortAudioClass pa;
  pa.Initialize();
  pa.OpenInputStream(sampleRate, framesPrBuffer, numChannels);
  pa.StartStream();

  int result;
  std::vector<int> fundneToner;

  decoder.setStartBit(false);
  bool shutdown = false;
  bool correctMessage = false;

  // Testing
  int expiredCount = 0;
  int ackCount = 0;
  int checksumFailCount = 0;
  std::vector<int> allTones;

  auto start = std::chrono::high_resolution_clock::now();

  while (!shutdown) {
    //   if(ackCount> 9
    // ){ 
    //     shutdown = true; 
    //      std::cout << "ACK Count: " << ackCount << std::endl; 
    //      std::cout << "Expired Count: " << expiredCount << std::endl; 
    //      std::cout << "Checksum Fail Count: " << checksumFailCount << std::endl; 
    //      std::cout << " 0: " << decoder.getCount(0) << std::endl; 
    //      std::cout << " 1: " << decoder.getCount(1) << std::endl; 
    //      std::cout << " 2: " << decoder.getCount(2) << std::endl; 
    //      std::cout << " 3: " << decoder.getCount(3) << std::endl; 
    //      std::cout << " 4: " << decoder.getCount(4) << std::endl; 
    //      std::cout << " 5: " << decoder.getCount(5) << std::endl; 
    //      std::cout << " 6: " << decoder.getCount(6) << std::endl; 
    //      std::cout << " 7: " << decoder.getCount(7) << std::endl; 
    //      std::cout << " 8: " << decoder.getCount(8) << std::endl; 
    //      std::cout << " 9: " << decoder.getCount(9) << std::endl; 
    //      std::cout << " A: " << decoder.getCount(10) << std::endl; 
    //      std::cout << " B: " << decoder.getCount(11) << std::endl; 
    //      std::cout << " C: " << decoder.getCount(12) << std::endl; 
    //      std::cout << " D: " << decoder.getCount(13) << std::endl; 
    //      std::cout << " *: " << decoder.getCount(14) << std::endl; 
    //      std::cout << " #: " << decoder.getCount(15) << std::endl; 
    //      std::cout << "Samplerate: " << sampleRate << std::endl; 
    //      std::cout << "Buffer size: " << framesPrBuffer << std::endl; 
    //      std::cout << "Tolerance: " << decoder.getTolerance() << std::endl; 
    //      std::cout << "threshold: " << decoder.getThreshold() << std::endl; 
    //   } 

    if (fundneToner.size() > 5) {
      decoder.setStartBit(false);
      decoder.clearLastSound();
      correctMessage = mi.interpretMessage(fundneToner);
      fundneToner.clear();

      if (correctMessage) {
        pa.StopStream();
        start = std::chrono::high_resolution_clock::now();

        pa.OpenOutputStream(44100, 1764, 1); // Open for playing
        pa.StartStream();

        // Play the acknowledgment tone (example: 697 Hz and 1209 Hz for 1
        // second)
        //Kan sættes ned til 20/20 eller 40/40
        //Hvis i gør det så husk at ændre buffer størrelsen i OpenOutputStream
        pa.PlayTone(697, 1209, 20, 20);
        pa.StopStream();

        ackCount++;
      } else {
        checksumFailCount++;
      }

      if (mi.getExecuteRoute()) {
        shutdown = true;
        // robo.commands(mi.getDriveCommands());
      }
      pa.StopStream();
      pa.OpenInputStream(sampleRate, framesPrBuffer, numChannels);
      pa.StartStream();
    } else if (std::chrono::high_resolution_clock::now() - start >
                   std::chrono::milliseconds(120) &&
               decoder.getStartBit()) {
      start = std::chrono::high_resolution_clock::now();
      decoder.setStartBit(false);
      fundneToner.clear();
      decoder.clearLastSound();
      std::cout << "Timer expired -> Cleared fundne Toner" << std::endl;
      expiredCount++;
    }

    std::vector<float> buffer;
    //  auto startTimeTest = std::chrono::high_resolution_clock::now();
    pa.ReadStream(buffer, framesPrBuffer);
    /* auto CurrentTimeTest = std::chrono::high_resolution_clock::now();
    auto elapsedTimeTest =
    std::chrono::duration_cast<std::chrono::milliseconds>(CurrentTimeTest -
    startTimeTest).count(); std::cout << "Tid om at fylde bufferen: " <<
    elapsedTimeTest << std::endl;
*/ result = decoder.FFT(buffer, sampleRate);

    if (result != 0 && result != 2277) {
      std::cout << result << std::endl;
    }

    if (result == 2277 && !decoder.getStartBit()) // tonen 0 og startbit = false
    {
      decoder.setStartBit(true);
      fundneToner.clear();
      start = std::chrono::high_resolution_clock::now();
      std::cout << "Start" << std::endl;
      continue;
    }

    else if (decoder.getStartBit() && result != 0) {
      fundneToner.push_back(result);
      allTones.push_back(result);
      start = std::chrono::high_resolution_clock::now();

      continue;
    }
  }
  std::cout << "ACK Count: " << ackCount << std::endl; 
  std::cout << "Expired Count: " << expiredCount << std::endl; 
  std::cout << "Checksum Fail Count: " << checksumFailCount << std::endl; 
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
