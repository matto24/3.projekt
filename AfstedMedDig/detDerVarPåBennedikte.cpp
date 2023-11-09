#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <portaudio.h>
#include <fftw3.h>
#include <cmath>
#include <map>
#include<unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "portaudio.h"

#define SAMPLE_RATE 8000
#define RECORDING_DURATION_SECONDS 0.2 // resolution = (sample_rate /(sample_rate*duration))
#define FRAMES_PER_BUFFER 1600
#define NUM_CHANNELS 1

using namespace std;

std::vector<double> DTMF1 = {697.5, 770, 852, 941};
std::vector<double> DTMF2 = {1209.5, 1336, 1477, 1633};

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
       // RCLCPP_INFO(this->get_logger(), "Publishing: %f , %f", this->msg.linear.x, this->msg.angular.z);
        publisher_->publish(msg);
    }

private:
    // Private variables used for the publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist msg;
};

// Vigtig note! i vores "int index" senere bliver det lavet til en integer or dermed afrundet.
// Med det antal samples N vi har og vores sample rate bliver
// 697.5 til index = 696. Det giver den største amplitude ligesom da vi i matlab så at den af en eller anden grund
// altid var størst ved 696 og ikke 697 som man skulle tro

int lastSound;
bool shutdown = false;

int FFT(const std::vector<double> &audioData, double sampleRate)
{
    int N = audioData.size();
    // std::cout << "N = " << N << std::endl;

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

    // Find største amp på ene led
    double largestAmp1 = avg / (N / 2 + 1) * 5;
    double largestAmp2 = largestAmp1;
    double largestFreq1 = 0;
    int sound = 0;

    for (double i : DTMF1)
    {
        int index = i * (N / sampleRate);
        double amp = std::sqrt(out[index][0] * out[index][0] + out[index][1] * out[index][1]);

        if (amp > largestAmp1)
        {
            largestAmp1 = amp;
            largestFreq1 = i;
        }
    }

    // Find største amp på anden led
    double largestFreq2 = 0;

    if (largestFreq1 != 0)
    {
        for (double i : DTMF2)
        {
            int index = i * (N / sampleRate);
            double amp = std::sqrt(out[index][0] * out[index][0] + out[index][1] * out[index][1]);

            if (amp > largestAmp2)
            {
                largestAmp2 = amp;
                largestFreq2 = i;
            }
        }
        if (largestFreq2 != 0)
        {
            // std::cout << "Largest DTMF1: " << largestFreq1 << " Largest DTMF2: " << largestFreq2 << std::endl;
            sound = largestFreq1 + largestFreq2;
            if (lastSound != sound)
            {
                lastSound = sound;
                fftw_destroy_plan(p);
                fftw_free(in);
                fftw_free(out);
                return sound;
            }
        }
    }
    // std::cout << "no valid DTMF found" << std::endl;
    //  De-allokerer hukommelse fra pointers.
    fftw_destroy_plan(p);
    fftw_free(in);
    fftw_free(out);

    return -1;
}

int main(int argc, char **argv)
{
    PaStream *stream;
    PaError err;
    int result;

    rclcpp::init(argc, argv);
    auto rb3_publisher = std::make_shared<RB3_cpp_publisher>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(rb3_publisher);

    // Initialize PortAudio
    err = Pa_Initialize();

    PaStreamParameters inputParameters;
    inputParameters.device = Pa_GetDefaultInputDevice();
    inputParameters.channelCount = NUM_CHANNELS;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultHighInputLatency;
    inputParameters.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(&stream, &inputParameters, nullptr, SAMPLE_RATE, FRAMES_PER_BUFFER, paClipOff, nullptr, nullptr);

    err = Pa_StartStream(stream);

    while (!shutdown)
    {

        std::vector<double> audioData;
        audioData.reserve(SAMPLE_RATE * NUM_CHANNELS * RECORDING_DURATION_SECONDS);

        for (int i = 0; i < RECORDING_DURATION_SECONDS * SAMPLE_RATE / FRAMES_PER_BUFFER; i++)
        {
            float buffer[FRAMES_PER_BUFFER];
            err = Pa_ReadStream(stream, buffer, FRAMES_PER_BUFFER);

            audioData.insert(audioData.end(), buffer, buffer + FRAMES_PER_BUFFER);
        }

        result = FFT(audioData, SAMPLE_RATE);

        switch (result)
        {
        case 1907:
            std::cout << "1" << std::endl;
            rb3_publisher->publish_vel(0.1, 0.2);
            break;
        case 2033:
            std::cout << "2" << std::endl;
            rb3_publisher->publish_vel(0.1, 0);

            break;
        case 2174:
            std::cout << "3" << std::endl;
            rb3_publisher->publish_vel(0.1, -0.2);
            break;
        case 2330:
            std::cout << "A" << std::endl;
            break;
        case 1979:
            std::cout << "4" << std::endl;
             rb3_publisher->publish_vel(0, 0.2);
            break;
        case 2106:
            std::cout << "5" << std::endl;
             rb3_publisher->publish_vel(0, 0);
            break;
        case 2247:
            std::cout << "6" << std::endl;
             rb3_publisher->publish_vel(0, -0.2);
            break;
        case 2403:
            std::cout << "B" << std::endl;
            break;
        case 2061:
            std::cout << "7" << std::endl;
            rb3_publisher->publish_vel(-0.1, -0.2);
            break;
        case 2188:
            std::cout << "8" << std::endl;
            rb3_publisher->publish_vel(-0.1, 0);
            break;
        case 2329:
            std::cout << "9" << std::endl;
            rb3_publisher->publish_vel(-0.1, 0.2);
            break;
        case 2485:
            std::cout << "C" << std::endl;
            break;
        case 2150:
            std::cout << "*" << std::endl;
            break;
        case 2277:
            std::cout << "0" << std::endl;
            break;
        case 2418:
            std::cout << "#" << std::endl;
            break;
        case 2574:
            std::cout << "D" << std::endl;
            shutdown = true;
            rclcpp::shutdown();
            break;
        default:
            break;
        }
    }

    err = Pa_StopStream(stream);

    err = Pa_CloseStream(stream);

    Pa_Terminate();
    return 0;
}
