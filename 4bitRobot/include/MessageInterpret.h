#pragma once

#include <vector>
#include <map>
#include <utility>
#include <string>


class MessageInterpreter {
public:
    MessageInterpreter();
    bool interpretMessage(const std::vector<char>& inputSekvens);
    bool getExecuteRoute();
    std::vector<std::pair<int, std::string>> getDriveCommands();

private:
    std::map<char, std::string> toneToBitMap;
    std::vector<std::pair<int, std::string>> driveCommands;
    bool executeRoute;
    std::string lastBits;
    volatile char selectedKey = '\0';
    volatile bool keepPlaying = false;
};

