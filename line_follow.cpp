#include "uart.hpp"
#include "libpixyusb2.h"
#include <iostream>
#include <string>




int main() {


    if (!uart::open()) { std::cerr << "open failed\n"; return 1; }

    Pixy2 pixy;

    int rc = pixy.init();
    if (rc < 0) { std::cerr << "pixy init failed: " << rc << "\n";
        uart::close(); return 2; }

    // Print firmware version
    std::cout << "FW: "
              << (int)pixy.version->firmwareMajor << "."
              << (int)pixy.version->firmwareMinor << "."
              << (int)pixy.version->firmwareBuild << "\n";
    

    uart::send("Test");
    std::string resp = uart::recv(256, 1000); // up to 256 bytes, 1s timeout
    if(resp.empty()) {
        std::cout << "no response\n";
    } else {
        std::cout << "UART active. Response: " << resp << "\n";
    }

    // Switch to line tracking mode
    rc = pixy.changeProg("line");
    if (rc < 0) { std::cerr << "changeProg failed: " << rc << "\n";}
    else { std::cout << "Line tracking mode active\n"; }

    //Turn on the lamps
    rc = pixy.setLamp(0, 0);
    if (rc < 0) std::cerr << "setLamp failed: " << rc << "\n";
    else std::cout << "Lamps on\n";

    // Set LED to green
    rc = pixy.setLED(0, 0, 0); // green
    if (rc < 0) std::cerr << "setLED failed: " << rc << "\n";
    else std::cout << "LED green\n";



    while (1) {
        int8_t feat = pixy.line.getMainFeatures();
        
        if (feat == 1) {
            std::cout << "Line detected!\n";
            
        }
        else if (feat == 2) {
            std::cout << "Intersection detected!\n";
        }
        else if (feat == PIXY_RESULT_BUSY) {
            // no new frame yet, just continue
            continue;
        }
        else if (feat < 0) {
            // handle other errors if you care
            std::cerr << "Error: " << (int)feat << "\n";
        }
    }


    

    uart::close();
    return 0;
}
