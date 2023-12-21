#include "FFT.h"

#include <vector>
#include <iostream>
#include <cmath>
#include <fftw3.h>
#include <array>
#include <algorithm>

DTMFDecoder::DTMFDecoder(int N) : DTMF1({697, 770, 852, 941}), 
                                  DTMF2({1209, 1336, 1477, 1633}),
                                  lastSound(0), tempSound(0), in(N), out(N / 2 + 1)
{
    plan = fftw_plan_dft_r2c_1d(N, in.data(), out.data(), FFTW_ESTIMATE);
    startBit = false;
}

DTMFDecoder::~DTMFDecoder()
{
    fftw_destroy_plan(plan);
}


void DTMFDecoder::setStartBit(bool in)
{
    startBit = in;
}

bool DTMFDecoder::getStartBit()
{
    return startBit;
}

void DTMFDecoder::clearLastSound()
{
    lastSound = 0;
}

int DTMFDecoder::FFT(const std::vector<float> &audioData, double sampleRate)
{
    int N = audioData.size();

    std::copy(audioData.begin(), audioData.end(), in.begin()); 

    // Execute the FFT plan
    fftw_execute(plan);
    double threshold = 2;
    double largestAmp1 = threshold;
    double largestAmp2 = threshold;
    double largestFreq1 = 0.0;
    double largestFreq2 = 0.0;
    int detectedSound = 0;

    // Function to check if a frequency is within tolerance of a target frequency
    auto isFrequencyInRange = [](double target, double actual, double tolerance)
    {
        return std::abs(target - actual) <= tolerance;
    };

    // Detection
    for (int i = 0; i <= N / 2; ++i)
    {
        double freq = i * sampleRate / N;
        double amp = std::sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]);

        for (double targetFreq : DTMF1)
        {
            if (isFrequencyInRange(targetFreq, freq, 25.0) && amp > largestAmp1)
            {
                largestAmp1 = amp;
                largestFreq1 = targetFreq; 
            }
        }

        for (double targetFreq : DTMF2)
        {
            if (isFrequencyInRange(targetFreq, freq, 25.0) && amp > largestAmp2)
            {
                largestAmp2 = amp;
                largestFreq2 = targetFreq; 
            }
        }
    }
    if (largestFreq2 != 0 && largestFreq1 != 0)
    {
        detectedSound = static_cast<int>(largestFreq1 + largestFreq2);

        if (lastSound == detectedSound)
        {
            return 0;
        }
        if (detectedSound == 2277 && startBit)
        {
            tempSound = lastSound;
            lastSound = detectedSound;
            return tempSound;
        }
        lastSound = detectedSound;
        return detectedSound;
    }
    return 0;
};
