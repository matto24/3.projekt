#include <SFML/Audio.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "sound.h"

//Ctrl + B to call makefile

 int main() { 
     std::vector<DtmfSound> sounds;
     //Initializing all sounds and storing them in a vector
     for(int i=0; i<16; i++) {
         DtmfSound sound(std::to_string(i));
         sounds.push_back(sound);
     }
     int x;
   std::cout << "Insert sound number here ";
     std::cin >> x;

    sounds[x].playSound();
    sounds[1].playSound();
    sounds[2].playSound();

     return 0;
 }