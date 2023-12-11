#include "FFT.h"

#include <vector>
#include <iostream>
#include <cmath>
#include <fftw3.h>
#include <array>
#include <algorithm>

DTMFDecoder::DTMFDecoder(int N) : DTMF1({697, 770, 852, 941}), // HUSK NU FOR GUDS SKYLD AT ÆNDRE ALLE TAL ALA 697.5
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

int DTMFDecoder::getCount(int i){
    return toneCount[i];
}
int DTMFDecoder::FFT(const std::vector<float> &audioData, double sampleRate)
{
    int N = audioData.size();
    // Copy the audio data to the input buffer
    std::copy(audioData.begin(), audioData.end(), in.begin()); // Måske lige gyldigt at kopiere til in når det ikk er FFTW Allocate?

    // Execute the FFT plan
    fftw_execute(plan);
    threshold = 5;
    tolerance = 25;
    // double threshold = abs(calculateAverageOfLast10Medians(audioData)*100); // LAV NOGET FEDT TIL THRESHOLD
    double largestAmp1 = threshold;
    double largestAmp2 = threshold;
    double largestFreq1 = 0.0;
    double largestFreq2 = 0.0;
    int detectedSound = 0;
    

    // Function to check if a frequency is within the range of a target frequency
    auto isFrequencyInRange = [](double target, double actual, double _tolerance)
    {
        return std::abs(target - actual) <= _tolerance;
    };

    // Adjusted detection logic
    for (int i = 0; i <= N / 2; ++i)
    {
        double freq = i * sampleRate / N;
        double amp = std::sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]);

        for (double targetFreq : DTMF1)
        {
            if (isFrequencyInRange(targetFreq, freq, tolerance) && amp > largestAmp1)
            {
                largestAmp1 = amp;
                largestFreq1 = targetFreq; // Use the target frequency, not the actual
            }
        }

        for (double targetFreq : DTMF2)
        {
            if (isFrequencyInRange(targetFreq, freq, tolerance) && amp > largestAmp2)
            {
                largestAmp2 = amp;
                largestFreq2 = targetFreq; // Use the target frequency, not the actual
            }
        }
    }
    if (largestFreq2 != 0 && largestFreq1 != 0)
    {

        std::cout << "largestAmp1: " << largestAmp1 << std::endl;
        std::cout << "largestAmp2: " << largestAmp2 << std::endl;
        detectedSound = static_cast<int>(largestFreq1 + largestFreq2);
        if (lastSound == detectedSound)
        {
            return 0;
        }
        switch (detectedSound)
        {
        case 2277:
            toneCount[0]++;
            break;
        case 1906:
            toneCount[1]++;
            break;
        case 2033:
            toneCount[2]++;
            break;
        case 2174:
            toneCount[3]++;
            break;
        case 1979:
            toneCount[4]++;
            break;
        case 2106:
            toneCount[5]++;
            break;
        case 2247:
            toneCount[6]++;
            break;
        case 2061:
            toneCount[7]++;
            break;
        case 2188:
            toneCount[8]++;
            break;
        case 2329:
            toneCount[9]++;
            break;
        case 2330:
            toneCount[10]++;
            break;
        case 2403:
            toneCount[11]++;
            break;
        case 2485:
            toneCount[12]++;
            break;
        case 2574:
            toneCount[13]++;
            break;
        case 2150:
            toneCount[14]++;
            break;
        case 2418:
            toneCount[15]++;
            break;
        default:
            break;
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

    // Find the largest frequency in the first DTMF group
    for (double freq : DTMF1)
    {
        int index = static_cast<int>(freq * (N / sampleRate));
        double amp = std::sqrt(out[index][0] * out[index][0] + out[index][1] * out[index][1]);

        if (amp > largestAmp1)
        {
            largestAmp1 = amp;
            largestFreq1 = freq;
        }
    }

    // If a frequency is found in the first group, find one in the second
    if (largestFreq1 != 0)
    {
        for (double freq : DTMF2)
        {
            int index = static_cast<int>(freq * (N / sampleRate));
            double amp = std::sqrt(out[index][0] * out[index][0] + out[index][1] * out[index][1]);

            if (amp > largestAmp2)
            {
                largestAmp2 = amp;
                largestFreq2 = freq;
            }
        }
        // If both frequencies are found, calculate the detected sound
        if (largestFreq2 != 0)
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
    }
    return 0;
};


double DTMFDecoder::getThreshold()
{
    return threshold;
}

double DTMFDecoder::getTolerance()
{
    return tolerance;
}
