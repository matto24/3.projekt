#pragma once
#include <SFML/Audio.hpp>
#include <string>

class DtmfSound {
    private: 
    std::string _x;

    public:
    DtmfSound(std::string str);
    void playSound();
   
};