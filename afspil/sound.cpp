#include "sound.h"
#include <iostream>
#include <string>
#include <SFML/Audio.hpp>

DtmfSound::DtmfSound(std::string str) {
    _x = str;
}

void DtmfSound::playSound() {
    sf::SoundBuffer _soundBuffer;
    std::string _soundLocation = "sounds/" + _x + ".wav";
    _soundBuffer.loadFromFile(_soundLocation);
    
    sf::Sound sound;
    sound.setBuffer(_soundBuffer);

    // Play the audio file
    sound.play();
    // Keep the program running to allow audio to play
    while (sound.getStatus() == sf::Sound::Playing) {
    }
}

