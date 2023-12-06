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

// Implement the calculateMedian function
double DTMFDecoder::calculateMedian(const std::vector<float> &vec)
{
    if (vec.empty())
    {
        std::cerr << "Error: Cannot calculate median of an empty vector." << std::endl;
        return 0.0;
    }

    // Sort the vector
    std::vector<float> sortedVec = vec;
    std::sort(sortedVec.begin(), sortedVec.end());

    // Calculate the median
    size_t size = sortedVec.size();
    size_t middle = size / 2;

    if (size % 2 == 0)
    {
        // If the size is even, take the average of the two middle elements
        return static_cast<double>(sortedVec[middle - 1] + sortedVec[middle]) / 2.0;
    }
    else
    {
        // If the size is odd, return the middle element
        return static_cast<double>(sortedVec[middle]);
    }
}

double DTMFDecoder::calculateAverageOfLast10Medians(const std::vector<float> &audioData)
{
    // Calculate the current median
    double currentMedian = calculateMedian(audioData);

    // Determine the current sound level (maximum value in audioData)
    float maxSoundLevel = *std::max_element(audioData.begin(), audioData.end());

    // Update the currentValue based on the sound level
    double currentValue = maxSoundLevel; // You can apply any scaling or transformation here

    // Add the current median to the list of last 10 medians
    this->last10Medians.push_back(currentMedian);

    // Keep only the last 5 medians
    if (this->last10Medians.size() > 5)
    {
        this->last10Medians.erase(this->last10Medians.begin());
    }

    // Calculate the average of the last 10 medians
    double sum = 0.0;
    for (double median : this->last10Medians)
    {
        sum += median;
    }

    // Calculate the average including the current value and a baseline
    double baseline = 2.0;                                                               // Replace this with your desired baseline value
    double average = (sum + currentValue + baseline) / (this->last10Medians.size() + 2); // 2 to account for currentValue and baseline

    return average;
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
    // Copy the audio data to the input buffer
    std::copy(audioData.begin(), audioData.end(), in.begin()); // Måske lige gyldigt at kopiere til in når det ikk er FFTW Allocate?

    // Execute the FFT plan
    fftw_execute(plan);
    double threshold = 260;
    // double threshold = abs(calculateAverageOfLast10Medians(audioData)*100); // LAV NOGET FEDT TIL THRESHOLD
    double largestAmp1 = threshold;
    double largestAmp2 = threshold;
    double largestFreq1 = 0.0;
    double largestFreq2 = 0.0;
    int detectedSound = 0;

    // Function to check if a frequency is within the range of a target frequency
    auto isFrequencyInRange = [](double target, double actual, double tolerance)
    {
        return std::abs(target - actual) <= tolerance;
    };

    // Adjusted detection logic
    for (int i = 0; i <= N / 2; ++i)
    {
        double freq = i * sampleRate / N;
        double amp = std::sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]);

        for (double targetFreq : DTMF1)
        {
            if (isFrequencyInRange(targetFreq, freq, 25.0) && amp > largestAmp1)
            {
                largestAmp1 = amp;
                largestFreq1 = targetFreq; // Use the target frequency, not the actual
            }
        }

        for (double targetFreq : DTMF2)
        {
            if (isFrequencyInRange(targetFreq, freq, 25.0) && amp > largestAmp2)
            {
                largestAmp2 = amp;
                largestFreq2 = targetFreq; // Use the target frequency, not the actual
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
