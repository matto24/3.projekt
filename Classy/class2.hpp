#pragma once

#include <vector>
#include <iostream>
#include <cmath>
#include <fftw3.h>
#include <array>

class DTMFDecoder {
private:
    const std::array<double, 4> DTMF1 = {697.5, 770, 852, 941};
    const std::array<double, 4> DTMF2 = {1209.5, 1336, 1477, 1633};
    int lastSound = 0;
    std::vector<double> in;
    std::vector<fftw_complex> out;
    fftw_plan plan;

public:
    explicit DTMFDecoder(size_t N) : in(N), out(N / 2 + 1) {
        plan = fftw_plan_dft_r2c_1d(N, in.data(), out.data(), FFTW_ESTIMATE);
    }

    ~DTMFDecoder() {
        fftw_destroy_plan(plan);
    }

    int FFT(const std::vector<double>& audioData, double sampleRate) {
        size_t N = audioData.size();
        //std::cout << N << std::endl;

        // Copy the audio data to the input buffer
        std::copy(audioData.begin(), audioData.end(), in.begin());

        // Execute the FFT plan
        fftw_execute(plan);

        double threshold = 150.0; // Adjust threshold based on your needs
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
                lastSound = detectedSound;
                return detectedSound;
            }
        }
        return 0;
    }
};
