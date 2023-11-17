#include <iostream>
#include <bitset>
#include <cmath>

// DECODER

class CommandDecoder {
public:
    static void decodeCommand(const std::string& encodedCommand) {
        // Extract parts of the encoded command
        std::string seqStr = encodedCommand.substr(0, 6);
        std::string cmdStr = encodedCommand.substr(6, 6);
        std::string dataStr = encodedCommand.substr(12, 15);
        char parityBit = encodedCommand.back();

        // Check parity
        if (checkParity(encodedCommand)) {
            // Convert binary strings to decimal values
            int sequenceNumber = std::stoi(seqStr, nullptr, 2);
            int commandCode = std::stoi(cmdStr, nullptr, 2);
            int dataValue = std::stoi(dataStr, nullptr, 2);

            // Output decoded values
            std::cout << "Decoded values:" << std::endl;
            std::cout << "Sequence Number: " << sequenceNumber << std::endl;
            std::cout << "Command Code: " << commandCode << std::endl;
            std::cout << "Data Value: " << dataValue << std::endl;

            // Play sound based on the command string
            playSound(encodedCommand);
        } else {
            std::cerr << "Parity check failed. Data may be corrupted." << std::endl;
        }
    }

private:
    static bool checkParity(const std::string& encodedCommand) {
        // Calculate parity bit (even parity)
        char calculatedParity = (encodedCommand.find('1') % 2 == 0) ? '1' : '0';
        return calculatedParity == encodedCommand.back();
    }

static void playSound(const std::string& commandString) {
    // Map each group of 3 bits to a sound string
    std::cout << "Play Sounds: ";
    for (size_t i = 0; i < commandString.size() - 1; i += 3) {
        std::string soundGroup = commandString.substr(i, 3);
        std::bitset<3> soundBits(soundGroup);
        int soundNumber = soundBits.to_ulong();
        std::cout << soundNumber;
    }
    std::cout << std::endl;
}
};

// GENERATOR

class CommandGenerator {
private:
    int sequenceNumber;

public:
    CommandGenerator() : sequenceNumber(0) {}

    std::string createCommand(int command, int data) {
        // Convert decimal values to binary strings
        std::bitset<6> seqBin(sequenceNumber);
        std::bitset<6> cmdBin(command);
        std::bitset<15> dataBin(data);

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

int main() {
    // Used for command generation
    CommandGenerator commandGenerator;
    int commandCode = 4;  // Replace with the actual code for "kør frem"
    int dataValue = 50;   // Replace with the actual data value (could be distance or angle)

    std::string command = commandGenerator.createCommand(commandCode, dataValue);
    std::cout << "Generated command: " << command << std::endl;

    // Used for command decoding
    // Decode the command
    CommandDecoder::decodeCommand(command);

    return 0;
}
