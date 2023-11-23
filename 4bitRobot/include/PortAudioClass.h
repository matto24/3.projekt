#pragma once

#include <portaudio.h>
#include <vector>

class PortAudioClass {
public:
    PortAudioClass();
    ~PortAudioClass();

    void Initialize();
    void OpenStream(int sampleRate, int framesPerBuffer, int numChannels);
    void StartStream();
    void ReadStream(std::vector<float>& buffer, int framesPerBuffer);
    void StopStream();

private:
    PaStream *stream;
    int sampleRate;
    int numChannels;
};
