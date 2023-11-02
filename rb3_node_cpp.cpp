#include <functional>
#include <memory>
#include <string>
#include <fftw3.h>
#include <iostream>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "portaudio.h"

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER 1024
#define NUM_CHANNELS 1
#define RECORDING_DURATION_SECONDS 1 // 0.1 virkede ok

std::vector<double> DTMF1 = {697.5, 770, 852, 941};
std::vector<double> DTMF2 = {1209.5, 1336, 1477, 1633};

using namespace std;

class RB3_cpp_publisher : public rclcpp::Node
{
public:
    // Create publisher for publishing veloctiy commands to topic "cmd_vel"
    RB3_cpp_publisher()
        : Node("rb3_cpp_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    // Class function to be called when you want to publish velocity commands
    void publish_vel(float lin_vel_cmd, float ang_vel_cmd)
    {
        // Set angular velocity to desired value (ie. turning)
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = ang_vel_cmd;

        // Set linear velocity to desired value (ie. forward and backwards)
        msg.linear.x = lin_vel_cmd;
        msg.linear.y = 0;
        msg.linear.z = 0;
        RCLCPP_INFO(this->get_logger(), "Publishing: %f , %f", this->msg.linear.x, this->msg.angular.z);
        publisher_->publish(msg);
    }

private:
    // Private variables used for the publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist msg;
};

void FFT(const std::vector<double> &audioData, double sampleRate, int argc, char **argv)
{
    int N = audioData.size();
    std::cout << "N = " << N << std::endl;

    /// FFTW setup
    double *in;        // En pointer til en double som senere vil blive til array reelle tal til vores FFT input
    fftw_complex *out; // En pointer et komplekst tal, som senere bliver et array til FFT output
    fftw_plan p;

    // En 'plan' i FFTW er en forudbestemt strategi for, hvordan man hurtigt kan beregne FFT. Det forbereder den mest effektive måde at transformere data på.

    in = (double *)fftw_malloc(sizeof(double) * N);                        // Allokere et array af reelle tal af længden N til vores input
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
    // FFTW_ESTIMATE betyder at den estimere den hurtigste plan i stedet for at prøve lidt forskelligt af (eller noget)
    fftw_execute(p); // udfører planen

    double avg;

    for (int i = 0; i < N / 2 + 1; i++)
    {
        avg = avg + std::sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]);
    }

    // std::cout << out.size() << std::endl;
    // std::cout << "penis" << avg/(N/2 + 1)  << std::endl;

    // Find største amp på ene led
    double largestAmp1 = avg / (N / 2 + 1) * 10;
    double largestFreq1 = 0;

    for (double i : DTMF1)
    {
        int index = i * (N / sampleRate);

        double amp = std::sqrt(out[index][0] * out[index][0] + out[index][1] * out[index][1]);

        // Pt. printer den amp for hver frekvens i DTMF spektret for debugging
        // std::cout << i << "---------" << amp << std::endl;
        if (amp > largestAmp1)
        {
            largestAmp1 = amp;
            largestFreq1 = i;
        }
    }

    // Find største amp på anden led
    double largestAmp2 = avg / (N / 2 + 1) * 10;
    double largestFreq2 = 0;

    if (largestFreq1 != 0)
    {
        for (double i : DTMF2)
        {
            int index = i * (N / sampleRate);

            double amp = std::sqrt(out[index][0] * out[index][0] + out[index][1] * out[index][1]);

            // std::cout << i << "---------" << amp << std::endl;
            if (amp > largestAmp2)
            {
                largestAmp2 = amp;
                largestFreq2 = i;
            }
        }
        if (largestFreq2 != 0)
        {
            std::cout << "Largest DTMF1: " << largestFreq1 << std::endl;
            std::cout << "Largest DTMF2: " << largestFreq2 << std::endl;
        }
        else
        {
            std::cout << "no valid DTMF found" << std::endl;
        }
    }
    else
    {
        std::cout << "no valid DTMF found" << std::endl;
    }

    if (largestFreq1 == 697.5 && largestFreq2 == 1209.5)
    {
        // Initialize rclcpp
        rclcpp::init(argc, argv);

        auto rb3_publisher = std::make_shared<RB3_cpp_publisher>();

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(rb3_publisher);

        double x = 0.0;
        double y = 0.0;
        int i = 1;
        while (i == 1)
        {
            // std::cout << "insert velocity: ";
            // std::cin >> x;
            // std::cout << "insert angular vel: ";
            // std::cin >> y;
            std::cin >> x;
            if (x == 1)
            {
                i = 0;
            }
            rb3_publisher->publish_vel(0.1, y);
        }
        // spin gør at ros node bliver ved med at køre og checke efter events den abbonerer på.
        //  executor.spin();

        // Shutdown node when complete
        rclcpp::shutdown();
    }

    // De-allokerer hukommelse fra pointers.
    fftw_destroy_plan(p);
    fftw_free(in);
    fftw_free(out);
}

int main(int argc, char **argv)
{

    PaStream *stream;
    PaError err;

    // Initialize PortAudio
    err = Pa_Initialize();
    if (err != paNoError)
    {
        std::cerr << "Error initializing PortAudio: " << Pa_GetErrorText(err) << std::endl;
        return -1;
    }

    // Set up input parameters
    PaStreamParameters inputParameters;
    inputParameters.device = Pa_GetDefaultInputDevice();
    if (inputParameters.device == paNoDevice)
    {
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
        nullptr, // No output
        SAMPLE_RATE,
        FRAMES_PER_BUFFER,
        paClipOff,
        nullptr, // No callback
        nullptr  // No user data
    );

    if (err != paNoError)
    {
        std::cerr << "Error opening stream: " << Pa_GetErrorText(err) << std::endl;
        Pa_Terminate();
        return -1;
    }

    // Start the audio stream
    err = Pa_StartStream(stream);
    if (err != paNoError)
    {
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
    for (int i = 0; i < RECORDING_DURATION_SECONDS * SAMPLE_RATE / FRAMES_PER_BUFFER; i++)
    {
        float buffer[FRAMES_PER_BUFFER];
        err = Pa_ReadStream(stream, buffer, FRAMES_PER_BUFFER);
        if (err != paNoError)
        {
            std::cerr << "Error reading stream: " << Pa_GetErrorText(err) << std::endl;
            break;
        }

        // Append the recorded data to the vector
        audioData.insert(audioData.end(), buffer, buffer + FRAMES_PER_BUFFER);
    }

    // Stop and close the audio stream
    err = Pa_StopStream(stream);
    if (err != paNoError)
    {
        std::cerr << "Error stopping stream: " << Pa_GetErrorText(err) << std::endl;
    }

    err = Pa_CloseStream(stream);
    if (err != paNoError)
    {
        std::cerr << "Error closing stream: " << Pa_GetErrorText(err) << std::endl;
    }

    // Terminate PortAudio
    Pa_Terminate();

    std::cout << "Recording complete." << std::endl;

    FFT(audioData, SAMPLE_RATE, argc, argv);

    return 0;
}
