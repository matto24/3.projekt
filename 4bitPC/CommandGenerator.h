#pragma once
#include <iostream>
#include <bitset>
#include <string>


class CommandGenerator {
private:

public:
    CommandGenerator() {}

    std::string createCommand(int command, int data) {
        // Convert decimal values to binary strings
        std::bitset<2> cmdBin(command);
        std::string cmdB = cmdBin.to_string();
        std::bitset<8> dataBin(data);
        std::string dataB = dataBin.to_string();
        std::string L = "11";


        if(dataB.substr(0,4) == "0000"){
           dataB = "1111" + dataB.substr(3,4);
           L = "01";
        }

        if(dataB.substr(3,4) == "0000"){
            dataB = dataB.substr(0,4) + "1111";
            L = "10";
        }

        cmdB += L;

        //12 ,16 ,20
        int checkVal = stoi((cmdB+dataB),nullptr,2);
        std::bitset<9> check1(checkVal);
        std::string checksum = check1.to_string();

        checksum = checksum.substr(0,3) + "1" + checksum.substr(3,3) + "1" + checksum.substr(6,3) + "1" + checksum.substr(9,checksum.size());


        


       

        //int seqBin = std::stoi(sequenceNumber.toString(),nullptr,2);

        // Concatenate binary strings
        std::string commandString = "0000" + cmdB + dataB + checksum;

        // Calculate parity bit (even parity)
        //char parityBit = (commandString.find('1') % 2 == 0) ? '1' : '0';

        // Add parity bit to the end of the command string
        //commandString += parityBit;

        // Increment the sequence number for the next command
       

        return commandString;
    }
};