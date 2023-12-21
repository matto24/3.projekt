#include "PortAudioClass.h"
#include <iostream>
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

void PortAudioClass::PlayTone(double frequency1, double frequency2, double duration, double sleeptime) {
    int sampleRate = 44100; 
    std::vector<float> buffer;
    GenerateTone(buffer, frequency1, frequency2, sampleRate, duration, sleeptime);

    int framesPerBuffer = static_cast<int>(buffer.size());
    Pa_WriteStream(stream, buffer.data(), framesPerBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(duration+sleeptime)));

}
