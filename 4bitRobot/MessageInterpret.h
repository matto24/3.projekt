#pragma once

#include <vector>
#include <map>
#include <utility>


class MessageInterpreter {
public:
    MessageInterpreter();
    void interpretMessage(const std::vector<int>& inputSekvens);

private:
    std::map<int, std::string> toneToBitMap;
    int lastSequenceNumber;
    std::vector<std::pair<int, std::string>> driveCommands;
};

