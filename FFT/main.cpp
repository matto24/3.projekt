#include <iostream>
#include <vector>
#include <fftw3.h>
#include <cmath>

//For at teste med en csv fil
#include <sstream>
#include <fstream>

const double PI = 3.14159265358979323846;

std::vector<double> generateSineWave(float frequency, float amplitude, float sampleRate, int numSamples)
{
    std::vector<double> waveform(numSamples);

    for (int i = 0; i < numSamples; i++)
    {
        waveform[i] = amplitude * std::sin(2 * PI * frequency * i / sampleRate);
    }

    return waveform;
}

std::vector<double> DTMF1 = {697.5, 770, 852, 941};
std::vector<double> DTMF2 = {1209.5, 1336, 1477, 1633};

// Vigtig note! i vores "int index" senere bliver det lavet til en integer or dermed afrundet. 
// Med det antal samples N vi har og vores sample rate bliver
// 697.5 til index = 696. Det giver den største amplitude ligesom da vi i matlab så at den af en eller anden grund
// altid var størst ved 696 og ikke 697 som man skulle tro

void FFT(const std::vector<double> &audioData, double sampleRate)
{
    int N = audioData.size();
    std::cout << "N = " << N << std::endl;

    /// FFTW setup
    double *in;     //En pointer til en double som senere vil blive til array reelle tal til vores FFT input
    fftw_complex *out;  //En pointer et komplekst tal, som senere bliver et array til FFT output
    fftw_plan p;

    // En 'plan' i FFTW er en forudbestemt strategi for, hvordan man hurtigt kan beregne FFT. Det forbereder den mest effektive måde at transformere data på.

    in = (double *)fftw_malloc(sizeof(double) * N); // Allokere et array af reelle tal af længden N til vores input
    out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * (N / 2 + 1)); // Ligeledes bare til komplekse output tal
    
    // Kun N/2 + 1 tal fordi vi laver en real to complex FFTW (Fordi der kun reelle tal i lyd og det her er vidst hurtigere)
    // Noget med at noget information er redundant (:)

    // Kopiere vores data til input variablen. Så vi sikre på at FFTW biblioteket har det hele gemt rigtigt i memory
    for (int i = 0; i < N; i++)
    {
        in[i] = audioData[i];
    }

    // Her oprettes først planen for vores FFT, derefter udføres den.
    p = fftw_plan_dft_r2c_1d(N, in, out, FFTW_ESTIMATE); // BEMÆRK! r2c (real to complex). 
    //FFTW_ESTIMATE betyder at den estimere den hurtigste plan i stedet for at prøve lidt forskelligt af (eller noget)
    fftw_execute(p); //udfører planen

    double avg;

    for(int i = 0; i < N/2+1; i++){
        avg = avg + std::sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]);
    }
    
    //std::cout << out.size() << std::endl;
    //std::cout << "penis" << avg/(N/2 + 1)  << std::endl;

    // Find største amp på ene led
    double largestAmp1 = avg/(N/2 + 1)*10;
    double largestFreq1 = 0;

    for (double i : DTMF1)
    {        
        int index = i * (N / sampleRate);

        double amp = std::sqrt(out[index][0] * out[index][0] + out[index][1] * out[index][1]);
        
        // Pt. printer den amp for hver frekvens i DTMF spektret for debugging
        //std::cout << i << "---------" << amp << std::endl;
        if (amp > largestAmp1){
            largestAmp1 = amp;
            largestFreq1 = i;
        }
    }
    
    // Find største amp på anden led
    double largestAmp2 = avg/(N/2 + 1)*10;
    double largestFreq2 = 0;

    if(largestFreq1 != 0){
    for (double i : DTMF2)
    {
        int index = i * (N / sampleRate);

        double amp = std::sqrt(out[index][0] * out[index][0] + out[index][1] * out[index][1]);
        
        //std::cout << i << "---------" << amp << std::endl;
        if (amp > largestAmp2){
            largestAmp2 = amp;
            largestFreq2 = i;
        }
    }
        if(largestFreq2 != 0){
        std::cout << "Largest DTMF1: " << largestFreq1 << std::endl;
        std::cout << "Largest DTMF2: " << largestFreq2 << std::endl;
        }
        else {
            std::cout << "no valid DTMF found" << std::endl;
        }
    } else {
        std::cout << "no valid DTMF found" << std::endl;
    }


    // De-allokerer hukommelse fra pointers. 
    fftw_destroy_plan(p);
    fftw_free(in);
    fftw_free(out);
}

int main(int argc, char const *argv[])
{
    std::cout << "YOLO" << std::endl;
    
    double sampleRate = 44100;        // Sample rate in Hz (e.g., 44100Hz is common for audio)
    int numSamples = sampleRate * 3; // Generate 1 second of audio

    std::vector<double> waveform1 = generateSineWave(697.5, 3.009, sampleRate, numSamples);
    std::vector<double> waveform2 = generateSineWave(770.5, 2.5, sampleRate, numSamples);
    std::vector<double> waveform3 = generateSineWave(941.5, 2.0901, sampleRate, numSamples);
    std::vector<double> waveform4 = generateSineWave(1209, 2.0901, sampleRate, numSamples);
    std::vector<double> waveform5 = generateSineWave(1477.5, 3.0901, sampleRate, numSamples);
    std::vector<double> waveform6 = generateSineWave(1633, 2.1901, sampleRate, numSamples);


    std::vector<double> result(waveform1.size());
    for (size_t i = 0; i < waveform1.size(); i++)
    {
        result[i] = waveform1[i] + waveform2[i] + waveform3[i] + waveform4[i] + waveform5[i] + waveform6[i];
    };

    //FFT(result, sampleRate);
    

    // CSV test
    std::vector<double> data;

    // Open the CSV file
    std::ifstream file("output.csv");

    std::string line;

    // Read each line from the CSV file
    while (std::getline(file, line)) {
        // Use stringstream to split the line into values
        std::istringstream ss(line);
        std::string value_str;
        
        while (std::getline(ss, value_str, ',')) {
            // Convert the string to a double and add it to the vector
            double value = std::stod(value_str);
            data.push_back(value);
        }
    }

    // Close the file
    file.close();

    FFT(data, 44100);

    return 0;
}
