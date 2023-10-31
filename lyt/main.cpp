#include <iostream>
#include <vector>
#include <portaudio.h>
#include <fstream> // Include the ofstream header for file writing
#include <fftw3.h>
#include <cmath>

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER 1024
#define NUM_CHANNELS 1
#define RECORDING_DURATION_SECONDS 1    //0.1 virkede ok
#define OUTPUT_FILE "output.csv"


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

int main() {
    PaStream *stream;
    PaError err;

    // Initialize PortAudio
    err = Pa_Initialize();
    if (err != paNoError) {
        std::cerr << "Error initializing PortAudio: " << Pa_GetErrorText(err) << std::endl;
        return -1;
    }

    // Set up input parameters
    PaStreamParameters inputParameters;
    inputParameters.device = Pa_GetDefaultInputDevice();
    if (inputParameters.device == paNoDevice) {
        std::cerr << "Error: No default input device." << std::endl;
        Pa_Terminate();
        return -1;
    }
    inputParameters.channelCount = NUM_CHANNELS;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultHighInputLatency;
    inputParameters.hostApiSpecificStreamInfo = nullptr;

    // Open the audio stream
    err = Pa_OpenStream(
        &stream,
        &inputParameters,
        nullptr,  // No output
        SAMPLE_RATE,
        FRAMES_PER_BUFFER,
        paClipOff,
        nullptr,  // No callback
        nullptr   // No user data
    );

    if (err != paNoError) {
        std::cerr << "Error opening stream: " << Pa_GetErrorText(err) << std::endl;
        Pa_Terminate();
        return -1;
    }

    // Start the audio stream
    err = Pa_StartStream(stream);
    if (err != paNoError) {
        std::cerr << "Error starting stream: " << Pa_GetErrorText(err) << std::endl;
        Pa_CloseStream(stream);
        Pa_Terminate();
        return -1;
    }

    std::cout << "Recording..." << std::endl;

    // Create a vector to store the recorded audio data
    std::vector<double> audioData;
    audioData.reserve(SAMPLE_RATE * NUM_CHANNELS * RECORDING_DURATION_SECONDS);

    // Record for the specified duration
    for (int i = 0; i < RECORDING_DURATION_SECONDS * SAMPLE_RATE / FRAMES_PER_BUFFER; i++) {
        float buffer[FRAMES_PER_BUFFER];
        err = Pa_ReadStream(stream, buffer, FRAMES_PER_BUFFER);
        if (err != paNoError) {
            std::cerr << "Error reading stream: " << Pa_GetErrorText(err) << std::endl;
            break;
        }

        // Append the recorded data to the vector
        audioData.insert(audioData.end(), buffer, buffer + FRAMES_PER_BUFFER);
    }

    // Stop and close the audio stream
    err = Pa_StopStream(stream);
    if (err != paNoError) {
        std::cerr << "Error stopping stream: " << Pa_GetErrorText(err) << std::endl;
    }

    err = Pa_CloseStream(stream);
    if (err != paNoError) {
        std::cerr << "Error closing stream: " << Pa_GetErrorText(err) << std::endl;
    }

    // Terminate PortAudio
    Pa_Terminate();

    std::cout << "Recording complete." << std::endl;

    // Use ofstream to save the audio data to a CSV file
    std::ofstream outputFile(OUTPUT_FILE);
    if (!outputFile.is_open()) {
        std::cerr << "Error opening output file: " << OUTPUT_FILE << std::endl;
        return -1;
    }

    // Write the audio data to the CSV file
    for (size_t i = 0; i < audioData.size(); i++) {
        outputFile << audioData[i];
        if (i < audioData.size() - 1) {
            outputFile << ",";
        }
    }

    FFT(audioData, SAMPLE_RATE);

    outputFile.close();

    std::cout << "Audio saved to " << OUTPUT_FILE << std::endl;

    return 0;
}
