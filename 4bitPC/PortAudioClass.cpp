#include "PortAudioClass.h"
#include <iostream>
#include <map>
#include <thread>
#include <chrono>

PortAudioClass::PortAudioClass() : stream(nullptr) {
}

PortAudioClass::~PortAudioClass() {
    StopStream();
    Pa_Terminate();
}

void PortAudioClass::Initialize() {
    PaError err = Pa_Initialize();
    
}

void PortAudioClass::OpenInputStream(int sampleRate, int framesPerBuffer, int numChannels) {
    PaStreamParameters inputParameters;
    inputParameters.device = Pa_GetDefaultInputDevice();
    inputParameters.channelCount = numChannels;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultHighInputLatency;
    inputParameters.hostApiSpecificStreamInfo = nullptr;

    PaError err = Pa_OpenStream(&stream, &inputParameters, nullptr, sampleRate, framesPerBuffer, paClipOff, nullptr, nullptr);
}

void PortAudioClass::OpenOutputStream(int sampleRate, int framesPerBuffer, int numChannels) {
    PaStreamParameters outputParameters;
    outputParameters.device = Pa_GetDefaultOutputDevice();
    outputParameters.channelCount = numChannels;
    outputParameters.sampleFormat = paFloat32;
    outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = nullptr;

    Pa_OpenStream(&stream, nullptr, &outputParameters, sampleRate, framesPerBuffer, paClipOff, nullptr, nullptr);
}

void PortAudioClass::StartStream() {
    PaError err = Pa_StartStream(stream);
}


void PortAudioClass::StopStream() {
    if (stream) {
        Pa_StopStream(stream);
        Pa_CloseStream(stream);
        stream = nullptr;
    }
}

void PortAudioClass::ReadStream(std::vector<float>& buffer, int framesPerBuffer) {
    buffer.resize(framesPerBuffer);
    PaError err = Pa_ReadStream(stream, buffer.data(), framesPerBuffer);
}

std::vector<double> PortAudioClass::keyToFrequencies(char key){
    
    std::vector<double> rowTones = {697, 770, 852, 941};
    std::vector<double> colTones = {1209, 1336, 1477, 1633};

    int row, col;

    switch (key)
    {
    case '1':
        row = 0;
        col = 0;
        break;
    case '2':
        row = 0;
        col = 1;
        break;
    case '3':
        row = 0;
        col = 2;
        break;
    case 'A':
        row = 0;
        col = 3;
        break;

    case '4':
        row = 1;
        col = 0;
        break;
    case '5':
        row = 1;
        col = 1;
        break;
    case '6':
        row = 1;
        col = 2;
        break;
    case 'B':
        row = 1;
        col = 3;
        break;

    case '7':
        row = 2;
        col = 0;
        break;
    case '8':
        row = 2;
        col = 1;
        break;
    case '9':
        row = 2;
        col = 2;
        break;
    case 'C':
        row = 2;
        col = 3;
        break;

    case '*':
        row = 3;
        col = 0;
        break;
    case '0':
        row = 3;
        col = 1;
        break;
    case '#':
        row = 3;
        col = 2;
        break;
    case 'D':
        row = 3;
        col = 3;
        break;

    default:
        row = -1;
        col = -1;
        break;
    }

    return {rowTones[row],colTones[col]};
}




void PortAudioClass::GenerateTone(std::vector<float>& buffer, double frequency1, double frequency2, double sampleRate, double playDuration, double sleepDuration) {

    int totalFrames = static_cast<int>((playDuration * sampleRate)/1000);
    int sleepFrames = static_cast<int>((sleepDuration*sampleRate)/1000);
    buffer.resize(totalFrames+sleepFrames);


    for (int i = 0; i < totalFrames; ++i) {
        double sample = (sin(2 * M_PI * frequency1 * i / sampleRate) + sin(2 * M_PI * frequency2 * i / sampleRate)) / 2;
        buffer[i] = static_cast<float>(sample);
    }

    for (int i = totalFrames; i < totalFrames+sleepFrames; ++i) {
        double sample = 0;
        buffer[i] = static_cast<float>(sample);
    }

}

void PortAudioClass::PlayTone(char key, double duration, double sleeptime) {
    int sampleRate = 44100; // Standard Sample Rate
    std::vector<double> freqs = keyToFrequencies(key);
    std::vector<float> buffer;
    GenerateTone(buffer, freqs[0], freqs[1], sampleRate, duration, sleeptime);

    int framesPerBuffer = static_cast<int>(buffer.size());
    Pa_WriteStream(stream, buffer.data(), framesPerBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(duration+sleeptime)));
}

std::string PortAudioClass::toneList(std::string binaryNum)
{
    std::map<std::string, std::string> toneToBitMap = {{"0", "0000"}, {"1", "0001"}, {"2", "0010"}, {"3", "0011"}, {"4", "0100"}, {"5", "0101"}, {"6", "0110"}, {"7", "0111"}, {"8", "1000"}, {"9", "1001"}, {"A", "1010"}, {"B", "1011"}, {"C", "1100"}, {"D", "1101"}, {"*", "1110"}, {"#", "1111"}};
    std::string output;

    // Iterator
    std::map<std::string, std::string>::iterator itr;
    for (int i = 0; i < binaryNum.size() / 4; i++)
    {
        //Det er faktisk de første 4
        std::string firstThree = binaryNum.substr(4 * i, 4);
        for (itr = toneToBitMap.begin(); itr != toneToBitMap.end(); ++itr)
        {
            if (itr->second == firstThree)
            {
                output += itr->first;
                break;
            }
        }
    }

    return output;
}
