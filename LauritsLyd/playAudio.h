#include <string>
#include <portaudio.h>

struct ThreadArgs {
    PaStream* stream;
    volatile char* selectedKey;
    volatile bool* keepPlaying;
};

class PlayAudio {
    private:
#define SAMPLE_RATE 44100
#define PI 3.14159265358979323846
#define AMPLITUDE 0.5
#define TONE_DURATION 0.1

// DTMF Frequencies (Hz)
static const double dtmfFrequencies[4][4];
    public:
    PlayAudio();
    static void generateDTMFTone(char key, float* buffer, int frames);
    static void* audioThread(void* args);
    std::string toneList(std::string binaryNum);
};