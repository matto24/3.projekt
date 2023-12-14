#pragma once
#include <iostream>
#include <bitset>
#include <string>


class CommandGenerator {
private:

public:
    CommandGenerator() {}

    std::string createCommand(int command, int data) {
        
        
        std::bitset<2> cmdBin(command);
        std::cout << cmdBin.to_string() << std::endl;
        std::string cmdB = cmdBin.to_string();
        std::bitset<8> dataBin(data);
        std::string dataB = dataBin.to_string();
        std::string L = "11";

        if(cmdB == "00"){
            dataB = "11100111";
        }

        if(dataB.substr(0,4) == "0000"){
           dataB = "1111" + dataB.substr(4,4);
           L = "01";
        }

        if(dataB.substr(4,4) == "0000"){
            dataB = dataB.substr(0,4) + "1111";
            if(L=="01"){
                L = "00";
            }
            else{
                L = "10";
            }
           
        }
        std::cout << "L: " << L << std::endl;
        cmdB += L;

        
        int checkVal = stoi((cmdB),nullptr,2) + stoi((dataB),nullptr,2);
        std::bitset<9> check1(checkVal);
        std::string checksum = check1.to_string();

        checksum = "1" + checksum.substr(0,3) + "1" + checksum.substr(3,3) + "1"+ checksum.substr(6,3);


      
        std::string commandString = "0000" + cmdB + dataB + checksum;

        // Calculate parity bit (even parity)
        //char parityBit = (commandString.find('1') % 2 == 0) ? '1' : '0';

        // Add parity bit to the end of the command string
        //commandString += parityBit;

        // Increment the sequence number for the next command
       
        //std::cout << cmdB << " - " <<dataB <<" - " << checksum << std::endl;
        return commandString;
    }
};