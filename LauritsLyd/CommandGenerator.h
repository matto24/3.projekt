#pragma once
#include <iostream>
#include <bitset>
#include <string>

class CommandGenerator {
private:
    int sequenceNumber;

public:
    CommandGenerator() : sequenceNumber(1) {}

    std::string createCommand(int command, int data) {
        // Convert decimal values to binary strings
        
        std::bitset<6> seqBin(sequenceNumber);
        std::bitset<6> cmdBin(command);

        std::string binaryDataString;
        int binaryData[32];
        int i=0;
        while(data > 0) {
            binaryData[i] = data%2;
            data = data/2;
            i++;
        }
        for(int j=i-1; j>=0; j--) {
            binaryDataString += std::to_string(binaryData[j]);
        }

        //int seqBin = std::stoi(sequenceNumber.toString(),nullptr,2);

        // Concatenate binary strings
        std::string commandString = seqBin.to_string() + cmdBin.to_string() + binaryDataString;

        // Calculate parity bit (even parity)
        char parityBit = (commandString.find('1') % 2 == 0) ? '1' : '0';

        // Add parity bit to the end of the command string
        commandString += parityBit;

        // Increment the sequence number for the next command
        sequenceNumber++;

        return commandString;
    }
};