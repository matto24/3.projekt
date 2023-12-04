#include "FFT.h"

#include <vector>
#include <iostream>
#include <cmath>
#include <fftw3.h>
#include <array>
#include <algorithm>
#include <map>

DTMFDecoder::DTMFDecoder(int N) : DTMF1({697, 770, 852, 941}), // HUSK NU FOR GUDS SKYLD AT ÆNDRE ALLE TAL ALA 697.5
                                  DTMF2({1209, 1336, 1477, 1633}),
                                  lastSound(0), tempSound(0), in(N), out(N / 2 + 1), lastTone('p')
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
    lastTone = 0;
    lastSound = 0;
}

char DTMFDecoder::FFT(const std::vector<float> &audioData, double sampleRate)
{
    int N = audioData.size();

    std::map<std::pair<int, int>, char> toneMap = {
        {{1212, 689}, '1'},
        {{1331, 689}, '2'},
        {{1474, 689}, '3'},
        {{1641, 689}, 'A'},
        {{1212, 761}, '4'},
        {{1331, 761}, '5'},
        {{1474, 761}, '6'},
        {{1641, 761}, 'B'},
        {{1212, 856}, '7'},
        {{1331, 856}, '8'},
        {{1474, 856}, '9'},
        {{1641, 856}, 'C'},
        {{1212, 951}, '*'},
        {{1331, 951}, '0'},
        {{1331, 927}, '0'},
        {{1474, 951}, '#'},
        {{1617, 951}, 'D'},
        {{1641, 951}, 'D'},
        {{1664, 951}, 'D'}
    };

    // Copy the audio data to the input buffer
    std::copy(audioData.begin(), audioData.end(), in.begin()); // Måske lige gyldigt at kopiere til in når det ikk er FFTW Allocate?

    // Execute the FFT plan
    fftw_execute(plan);
    double threshold = 60;
    // double threshold = abs(calculateAverageOfLast10Medians(audioData)*100); // LAV NOGET FEDT TIL THRESHOLD

    double largestAmp1 = 0;
    double largestAmp2 = 0;
    double largestFreq1 = 0;
    double largestFreq2 = 0;
    int detectedSound = 0;

    std::vector<double> amplitudes;
    double current;

    for (int index = 0; index < out.size(); index++)
    {
        current = std::sqrt(out[index][0] * out[index][0] + out[index][1] * out[index][1]);

        if (current > largestAmp1 && index * sampleRate / N < 1000)
        {
            largestFreq1 = index;
            largestAmp1 = current;
        }
        if (current > largestAmp2 && index * sampleRate / N > 1000)
        {
            largestFreq2 = index;
            largestAmp2 = current;
        }
    }

    int a = largestFreq1 * sampleRate / N;
    int b = largestFreq2 * sampleRate / N;


    //std::cout << "a: " << a << " b: " << b << std::endl;
    //    std::cout << largestFreq1 * sampleRate / N << "|" << largestFreq2 * sampleRate / N << std::endl;

    char c = toneMap[{b, a}];

    if(c == NULL){
        return 'x';
    }
    if(c == lastTone){
        return 'x';
    }
    if (c == '0' && startBit){
        char temp = lastTone;
        lastTone = c;
        return temp;
    }
    //std::cout << "| " << largestAmp1 << " | " << largestAmp2 << " |" << std::endl;
    lastTone = c;
    return c;
} /*
    if (c != NULL)
    {
        if (c != lastTone)
        {

            if (c == '0' && startBit)
            {
                //std::cout << "jeg inde i den her" << std::endl;
                char temp = lastTone;
                lastTone = c;
                return temp;
            }
            //std::cout << "jeg kom ud igen" << std::endl;
            lastTone = c;
            return c;
        }
        return 0;
    }

    return 0;
}

/*
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
*/ 