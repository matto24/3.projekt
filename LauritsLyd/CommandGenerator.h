#pragma once
#include<iostream>
#include<bitset>

class CommandGenerator {
private:
    int sequenceNumber;

public:
    CommandGenerator() : sequenceNumber(1) {}

    std::string createCommand(int command, int data) {
        // Convert decimal values to binary strings
        
        std::bitset<6> seqBin(sequenceNumber);
        std::bitset<6> cmdBin(command);
        std::bitset<8> dataBin(data);
        //int seqBin = std::stoi(sequenceNumber.toString(),nullptr,2);
    

        // Concatenate binary strings
        std::string commandString = seqBin.to_string() + cmdBin.to_string() + dataBin.to_string();

        // Calculate parity bit (even parity)
        char parityBit = (commandString.find('1') % 2 == 0) ? '1' : '0';

        // Add parity bit to the end of the command string
        commandString += parityBit;

        // Increment the sequence number for the next command
        sequenceNumber++;

        return commandString;
    }
};