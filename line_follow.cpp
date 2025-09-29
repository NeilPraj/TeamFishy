#include "uart.hpp"
#include "libpixyusb2.h"
#include <iostream>
#include <string>


int main() {
    
    if (!uart::open()) { std::cerr << "open failed\n"; return 1; }

    Pixy2 pixy;

    int r = pixy.init();
    if (r < 0) { std::cerr << "pixy init failed: " << r << "\n";
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

    
    
    while(1){
        int8_t feat = pixy.line.getMainFeatures(false);
        std::cout << "feat: " << (int)feat << "\n";
    }

    

    uart::close();
    return 0;
}
