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
    outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultHighOutputLatency;
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

void PortAudioClass::GenerateTone(std::vector<float>& buffer, double frequency1, double frequency2, double sampleRate, double duration) {
    int totalFrames = static_cast<int>(duration * sampleRate);
    buffer.resize(totalFrames);

    for (int i = 0; i < totalFrames; ++i) {
        double sample = (sin(2 * M_PI * frequency1 * i / sampleRate) + sin(2 * M_PI * frequency2 * i / sampleRate)) / 2;
        buffer[i] = static_cast<float>(sample);
    }
}

void PortAudioClass::PlayTone(double frequency1, double frequency2, double duration) {
    int sampleRate = 44100; // Standard Sample Rate
    std::vector<float> buffer;
    GenerateTone(buffer, frequency1, frequency2, sampleRate, duration);

    int framesPerBuffer = static_cast<int>(buffer.size());
    Pa_WriteStream(stream, buffer.data(), framesPerBuffer);
}