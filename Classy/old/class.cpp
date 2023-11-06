#include <vector>
#include <iostream>
#include <cmath>
#include <fftw3.h>

class DTMFDecoder {
private:
    std::vector<double> DTMF1 = {697.5, 770, 852, 941};
    std::vector<double> DTMF2 = {1209.5, 1336, 1477, 1633};
    int lastSound = 0;

public:
    DTMFDecoder() = default;
    ~DTMFDecoder() = default;

    int FFT(const std::vector<double>& audioData, double sampleRate) {
        int N = audioData.size();
        double *in;
        fftw_complex *out;
        fftw_plan p;
        
        in = (double *)fftw_malloc(sizeof(double) * N);
        out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * (N / 2 + 1));

        for (int i = 0; i < N; i++) {
            in[i] = audioData[i];
        }

        p = fftw_plan_dft_r2c_1d(N, in, out, FFTW_ESTIMATE);
        fftw_execute(p);

        double avg = 0.0;
        for (int i = 0; i < N / 2 + 1; i++) {
            avg += std::sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]);
        }

        double largestAmp1 = avg / (N / 2 + 1) * 5;
        double largestAmp2 = largestAmp1;
        double largestFreq1 = 0;
        int sound = 0;

        for (double i : DTMF1) {
            int index = i * (N / sampleRate);
            double amp = std::sqrt(out[index][0] * out[index][0] + out[index][1] * out[index][1]);

            if (amp > largestAmp1) {
                largestAmp1 = amp;
                largestFreq1 = i;
            }
        }

        double largestFreq2 = 0;

        if (largestFreq1 != 0) {
            for (double i : DTMF2) {
                int index = i * (N / sampleRate);
                double amp = std::sqrt(out[index][0] * out[index][0] + out[index][1] * out[index][1]);

                if (amp > largestAmp2) {
                    largestAmp2 = amp;
                    largestFreq2 = i;
                }
            }
            if (largestFreq2 != 0) {
                sound = largestFreq1 + largestFreq2;
                if (lastSound == sound) {
                    cleanup(p, in, out);
                    return lastSound;
                }
                lastSound = sound;
            }
        }

        cleanup(p, in, out);
        return sound;
    }

private:
    void cleanup(fftw_plan &p, double* in, fftw_complex* out) {
        fftw_destroy_plan(p);
        fftw_free(in);
        fftw_free(out);
    }
};

