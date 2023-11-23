#include "MessageInterpret.h"
#include <iostream>

MessageInterpreter::MessageInterpreter(){
    // Initialize the toneToBitMap
    executeRoute = false;
    toneToBitMap = {
    {2277, "0000"}, //0
    {1907, "0001"}, //1
    {2033, "0010"}, //2
    {2174, "0011"}, //3
    {1979, "0100"}, //4
    {2106, "0101"}, //5
    {2247, "0110"}, //6
    {2061, "0111"}, //7
    {2188, "1000"}, //8
    {2329, "1001"}, //9
    
    {2330, "1010"}, //A
    {2403, "1011"}, //B
    {2485, "1100"}, //C
    {2574, "1101"}, //D

    {2150, "1110"}, //*
    {2418, "1111"}, //#
    };
}

bool MessageInterpreter::getExecuteRoute(){
    return executeRoute;
}

std::vector<std::pair<int, std::string>> MessageInterpreter::getDriveCommands(){
    return driveCommands;
}

void MessageInterpreter::interpretMessage(const std::vector<int>& inputSekvens) {
        //Create string of bits
    std::string bits;
    for (int i = 0; i < inputSekvens.size(); i++)
    {
        bits = bits + toneToBitMap[inputSekvens[i]];
    }
    std::cout << bits << std::endl;

    /*//Parity Check:
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
    } */
    

    // Data
    std::string data = bits.substr(8, 8);
    if(bits[2] == '0'){
        data.replace(0,4, "0000");
    } else if(bits[3] == '0'){
        data.replace(4,4, "0000");
    }

    // Checksum
    std::string checksumTarget = bits.substr(13, 3) + bits.substr(17, 3) + bits.substr(21, 3);
    int checksumIntTarget = stoi(checksumTarget, nullptr, 2);
    int checksumIntCount = stoi(bits.substr(0, 4), nullptr, 2) + stoi(bits.substr(8, 8), nullptr, 2);

     //if (checksumIntCount != checksumIntTarget){
     //    return; 
     //}

        //Execute command
    int commandInt = stoi(bits.substr(0, 2), nullptr, 2);
    switch (commandInt) 
    {
    case 0b01: //Command-code
        std::cout << "drej til højre" << std::endl;
        driveCommands.push_back(std::make_pair(commandInt, data));
        break;
    case 0b10:
        std::cout << "drej til venstre" << std::endl;
        driveCommands.push_back(std::make_pair(commandInt, data));
        break;
    case 0b11: //Command-code
        std::cout << "Kør frem" << std::endl;
        driveCommands.push_back(std::make_pair(commandInt, data));
        break;
    case 0b00:
        std::cout << "Execute" << std::endl;
        executeRoute = true;
    default:
        break;
    }

}
