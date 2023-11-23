#include "PortAudioClass.h"
#include <iostream>

PortAudioClass::PortAudioClass() : stream(nullptr) {
}

PortAudioClass::~PortAudioClass() {
    StopStream();
    Pa_Terminate();
}

void PortAudioClass::Initialize() {
    PaError err = Pa_Initialize();
    
}

void PortAudioClass::OpenStream(int sampleRate, int framesPerBuffer, int numChannels) {
    PaStreamParameters inputParameters;
    inputParameters.device = Pa_GetDefaultInputDevice();
    inputParameters.channelCount = numChannels;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultHighInputLatency;
    inputParameters.hostApiSpecificStreamInfo = nullptr;

    PaError err = Pa_OpenStream(&stream, &inputParameters, nullptr, sampleRate, framesPerBuffer, paClipOff, nullptr, nullptr);
}

void PortAudioClass::StartStream() {
    PaError err = Pa_StartStream(stream);
}

void PortAudioClass::ReadStream(std::vector<float>& buffer, int framesPerBuffer) {
    buffer.resize(framesPerBuffer);
    PaError err = Pa_ReadStream(stream, buffer.data(), framesPerBuffer);
}

void PortAudioClass::StopStream() {
    if (stream) {
        Pa_StopStream(stream);
        Pa_CloseStream(stream);
        stream = nullptr;
    }
}
