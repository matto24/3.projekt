#pragma once

#include <vector>
#include <array>
#include <fftw3.h>

class DTMFDecoder {
private:
    const std::array<double, 4> DTMF1;
    const std::array<double, 4> DTMF2;
    int lastSound;
    int N;
    int tempSound;
    std::vector<double> in;
    std::vector<fftw_complex> out;
    fftw_plan plan;

public:
    explicit DTMFDecoder(int N);
    ~DTMFDecoder();

    int FFT(const std::vector<float>& audioData, double sampleRate);
};
