#include "MessageInterpret.h"
#include <iostream>

MessageInterpreter::MessageInterpreter(){
    // Initialize the toneToBitMap
    executeRoute = false;
    lastBits = "hej";
    toneToBitMap = {
    {'0', "0000"}, //0
    {'1', "0001"}, //1
    {'2', "0010"}, //2
    {'3', "0011"}, //3
    {'4', "0100"}, //4
    {'5', "0101"}, //5
    {'6', "0110"}, //6
    {'7', "0111"}, //7
    {'8', "1000"}, //8
    {'9', "1001"}, //9
    
    {'A', "1010"}, //A
    {'B', "1011"}, //B
    {'C', "1100"}, //C
    {'D', "1101"}, //D

    {'*', "1110"}, //*
    {'#', "1111"}, //#
    };
}

bool MessageInterpreter::getExecuteRoute(){
    return executeRoute;
}

std::vector<std::pair<int, std::string>> MessageInterpreter::getDriveCommands(){
    return driveCommands;
}

bool MessageInterpreter::interpretMessage(const std::vector<char>& inputSekvens) {
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
    int checksumIntCount = stoi(bits.substr(0, 4), nullptr, 2) + stoi(bits.substr(4, 8), nullptr, 2);


    if (checksumIntCount != checksumIntTarget){
         std::cout << "Forkert checksum" << std::endl;
         return false; 
     } else {
        std::cout << "rigtig checksum" << std::endl;
     }
     

     if(bits==lastBits) {
        return true;
     }

    lastBits = bits;

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

    return true;

}
