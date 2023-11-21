#include "FFT.h"

#include <vector>
#include <iostream>
#include <cmath>
#include <fftw3.h>
#include <array>


DTMFDecoder::DTMFDecoder(int N) : DTMF1({697.5, 770, 852, 941}),    // HUSK NU FOR GUDS SKYLD AT ÆNDRE ALLE TAL ALA 697.5
                                  DTMF2({1209.5, 1336, 1477, 1633}), 
                                  lastSound(0), tempSound(0), in(N), out(N / 2 + 1) {
    plan = fftw_plan_dft_r2c_1d(N, in.data(), out.data(), FFTW_ESTIMATE);
}

DTMFDecoder::~DTMFDecoder() {
        fftw_destroy_plan(plan);
    }

int DTMFDecoder::FFT(const std::vector<float>& audioData, double sampleRate) 
{
        int N = audioData.size();
        //std::cout << N << std::endl;

        // Copy the audio data to the input buffer
        std::copy(audioData.begin(), audioData.end(), in.begin()); //Måske lige gyldigt at kopiere til in når det ikk er FFTW Allocate?

        // Execute the FFT plan
        fftw_execute(plan);

        double threshold = 100; // LAV NOGET FEDT TIL THRESHOLD
        double largestAmp1 = threshold;
        double largestAmp2 = threshold;
        double largestFreq1 = 0.0;
        double largestFreq2 = 0.0;
        int detectedSound = 0;

        // Find the largest frequency in the first DTMF group
        for (double freq : DTMF1) {
            int index = static_cast<int>(freq * (N / sampleRate));
            double amp = std::sqrt(out[index][0] * out[index][0] + out[index][1] * out[index][1]);

            if (amp > largestAmp1) {
                largestAmp1 = amp;
                largestFreq1 = freq;
            }
        }

        // If a frequency is found in the first group, find one in the second
        if (largestFreq1 != 0) {
            for (double freq : DTMF2) {
                int index = static_cast<int>(freq * (N / sampleRate));
                double amp = std::sqrt(out[index][0] * out[index][0] + out[index][1] * out[index][1]);

                if (amp > largestAmp2) {
                    largestAmp2 = amp;
                    largestFreq2 = freq;
                }
            }
            // If both frequencies are found, calculate the detected sound
            if (largestFreq2 != 0) {
                detectedSound = static_cast<int>(largestFreq1 + largestFreq2);
                
                if (lastSound == detectedSound) {
                    return 0;
                }
                if (detectedSound == 2330){
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
