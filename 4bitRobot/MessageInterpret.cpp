#include "MessageInterpreter.h"
#include <iostream>

MessageInterpreter::MessageInterpreter() : lastSequenceNumber(0) {
    // Initialize the toneToBitMap
    toneToBitMap = {{2277, "000"}, {1907, "001"}, {2033, "010"}, {2174, "011"}, {1979, "100"}, {2106, "101"}, {2247, "110"}, {2061, "111"}, {2188, "0"}, {2329, "1"}};
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
    default:
        break;
    }

}
