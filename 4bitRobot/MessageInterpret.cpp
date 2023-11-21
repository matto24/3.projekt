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
    {2277, "1011"}, //B
    {2277, "1100"}, //C
    {2277, "1101"}, //D

    {2277, "1110"}, //*
    {2277, "1111"}, //#
 
    };
}

bool MessageInterpreter::getExecuteRoute(){
    return executeRoute;
}

std::vector<std::pair<int, std::string>> MessageInterpreter::getDrieCommands(){
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
    


        //Execute command
    int commandInt = stoi(bits.substr(6, 6), nullptr, 2);
    
    int dataFieldBitLength = bits.size() - 13; //13bits = 6 til seqNr., 6 til command og 1 til parity
    std::string data = bits.substr(12, dataFieldBitLength);

    switch (commandInt) 
    {
    case 0b111110: //Command-code
        driveCommands.push_back(std::make_pair(commandInt, data));
        break;
    case 0b101010:
        driveCommands.push_back(std::make_pair(commandInt, data));
        break;
    case 0b010111: //Command-code
        driveCommands.push_back(std::make_pair(commandInt, data));
        break;
    case 0b101110:
        driveCommands.push_back(std::make_pair(commandInt, data));
        break;
    case 0b0011:
        executeRoute = true;
    default:
        break;
    }

}
