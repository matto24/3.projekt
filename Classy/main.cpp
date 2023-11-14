#include <iostream>
#include <vector>
#include <map>

#include "FFT.h"
#include "PortAudioClass.h"
#include "rb3_cpp_publisher.h"
#include "drive.h"
#include <unistd.h>

const int sampleRate = 8000;
const double recordingDurationSeconds = 0.2; // resolution = (sample_rate /(sample_rate*duration))
const int framesPrBuffer = 1600;
const int numChannels = 1;

bool shutdown = false;

std::map<int, std::string> toneToBitMap = {{2277, "000"}, {1907, "001"}, {2033, "010"}, {2174, "011"}, {1979, "100"}, {2106, "101"}, {2247, "110"}, {2061, "111"}, {2188, "0"}, {2329, "1"}};

int lastSequenceNumber = 0;

void interpretMessage(std::vector<int> &inputSekvens, Drive *robo)
{
        //Create string of bits

    std::string bits;
    for (int i = 0; i < inputSekvens.size(); i++)
    {
        bits = bits + toneToBitMap[inputSekvens[i]];
    }
    std::cout << bits << std::endl;

        //Parity Check:

    int oneCount = 0;
    for (char c : bits)
    {
        if (c == '1'){
            oneCount++;
        }       
    }
    std::cout << oneCount << std::endl;
    if (bits.substr(bits.length() - 1) == "1" && !oneCount % 2 == 0)
    {
        std::cout << "parity fail" << std::endl;
        //return error?;
    }
    
        // Sequence number check

    int sequenceNumber = stoi(bits.substr(0, 6), nullptr, 2);

    std::cout << "seqNr: " << sequenceNumber << std::endl;

    if (sequenceNumber != lastSequenceNumber + 1)
    {
        lastSequenceNumber = 0; // eller hvad skal der ske ved fejl
        std::cout << "sequence number fail" << std::endl;
        //return error?;
    } 
    lastSequenceNumber = sequenceNumber;

        //Execute command
    int commandInt = stoi(bits.substr(6, 6), nullptr, 2);
    
    int dataFieldBitLength = bits.size() - 13; //13bits = 6 til seqNr., 6 til command og 1 til parity
    std::string data = bits.substr(12, dataFieldBitLength);

    switch (commandInt) 
    {
    case 0b111110: //Command-code
        robo->forwards(data);
        break;
    case 0b101010:
        robo->turnRight(data);
        break;
    case 0b010111: //Command-code
        robo.>backwards(data);
        break;
    case 0b101110:
        robo->turnleft(data);
        break;

    default:
        break;
    }

    shutdown = true;
}

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto rb3_publisher = std::make_shared<RB3_cpp_publisher>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(rb3_publisher);

    Drive robo(rb3_publisher);

    int result;
    DTMFDecoder decoder(1600);

    PortAudioClass pa;
    pa.Initialize();
    pa.OpenStream(sampleRate, framesPrBuffer, numChannels);
    pa.StartStream();

    const size_t ringBufferSize = sampleRate * numChannels * recordingDurationSeconds;
    std::vector<double> ringBuffer(ringBufferSize, 0.0);
    size_t ringBufferIndex = 0;
    std::vector<int> fundneToner;
    bool startBit = false;

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
                std::cout << result << std::endl;
            }

            if (result == 2150 && !startBit)
            {
                startBit = true;
                fundneToner.clear();
                std::cout << "start" << std::endl;

                continue;
            }

            else if (result == 2418 && startBit)
            { // "# - stopbit"
                std::cout << "end" << std::endl;
                interpretMessage(fundneToner, &robo);
                startBit = false;

                continue;
            }

            if (startBit && result != 0)
            {
                fundneToner.push_back(result);
            }
        }
        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double, std::milli> duration = end - start;
        // std::cout << "Processing cycle took " << duration.count() << " milliseconds.\n";
    }
    rclcpp::shutdown();
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
