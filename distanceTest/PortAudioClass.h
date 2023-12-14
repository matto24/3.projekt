#pragma once

#include <portaudio.h>
#include <vector>
#include <cmath>

class PortAudioClass {
public:
    PortAudioClass();
    ~PortAudioClass();

    void Initialize();
    void OpenInputStream(int sampleRate, int framesPerBuffer, int numChannels);
    void OpenOutputStream(int sampleRate, int framesPerBuffer, int numChannels);
    void StartStream();
    void StopStream();
    void ReadStream(std::vector<float>& buffer, int framesPerBuffer);
    void PlayTone(char key, double duration,double sleeptime);
    std::vector<double> keyToFrequencies(char key);

private:
    PaStream *stream;
    
    void GenerateTone(std::vector<float>& buffer, double frequency1, double frequency2, double sampleRate, double playDuration, double sleepDuration);
    int sampleRate;
    int numChannels;
};
