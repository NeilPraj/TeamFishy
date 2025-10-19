#include "uart.hpp"
#include "libpixyusb2.h"
#include "camera.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstring>
#include <csignal>


#define X_CENTER (pixy.frameWidth/2) // center of the Pixy2 camera's view (316 pixels wide)


static bool run_flag = true;


void handle_SIGINT(int unused);
void recv_float_with_timeout(uint8_t rxsize, uint32_t timeout_ms, float sent_val);

float kp = 2.0f;
float ki = 0.3f;
float kd = 0.2f;

// removes leading/trailing spaces, tabs, and newlines
static std::string trim(const std::string& s) {
    const auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos)
        return {};
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}


int main() {
    std::signal(SIGINT, handle_SIGINT);

    if (!uart::open("/dev/ttyS3", 115200)) { std::cerr << "open failed\n"; return 1; }
    else{
        std::cout << "UART opened\n";
    }

    Pixy2 pixy;
    setup_camera(pixy);


    

  
    /*
    uart::send(std::string_view(reinterpret_cast<const char*>(&kp), sizeof(kp)));
    recv_float_with_timeout(4, 1000, kp);
    uart::send(std::string_view(reinterpret_cast<const char*>(&ki), sizeof(ki)));
    recv_float_with_timeout(4, 1000, ki);
    uart::send(std::string_view(reinterpret_cast<const char*>(&kd), sizeof(kd)));
    recv_float_with_timeout(4, 1000, kd);
    */




    std::string rxbuf;
    rxbuf.reserve(512);


    while (run_flag) {
        std::string chunk = uart::recv(256, 200); // returns on timeout
        if (!chunk.empty()) {
            rxbuf.append(chunk);
            if (rxbuf.size() > 4096) rxbuf.erase(0, rxbuf.size() - 1024);

            for (;;) {
                if (!run_flag) break;

                size_t eol = rxbuf.find_first_of("\r\n");
                if (eol == std::string::npos) break;

                std::string raw = rxbuf.substr(0, eol);
                size_t next = rxbuf.find_first_not_of("\r\n", eol);
                rxbuf.erase(0, next == std::string::npos ? rxbuf.size() : next);

                std::string line = trim(raw);
                if (line.empty()) continue;

                // BIG IF CHAIN 
                if (line == "G kp") {
                    std::cout << "[CMD] get kp\n";
                    float val = kp;
                    uart::send(std::string_view(reinterpret_cast<const char*>(&val), sizeof(val)));
                } else if(line == "S kp"){
                    std::cout << "[RESP] SET KP\n";
                } else if(line == "G ki"){
                    std::cout << "[CMD] get ki\n";
                    float val = ki;
                    uart::send(std::string_view(reinterpret_cast<const char*>(&val), sizeof(val)));
                } else if(line == "S ki"){
                    std::cout << "[RESP] SET KI\n";
                } else if(line == "G kd"){
                    std::cout << "[CMD] get kd\n";
                    float val = kd;
                    uart::send(std::string_view(reinterpret_cast<const char*>(&val), sizeof(val)));
                } else if(line == "S kd"){
                    std::cout << "[RESP] SET KD\n";
                } 
                else if(line == "G H"){
                    std::cout << "[RESP] Get heading error\n";
                    int val = get_heading_error(pixy);
                    std::cout << "Heading error: " << val << "\n";
                    uart::send(std::string_view(reinterpret_cast<const char*>(&val), sizeof(val)));
                } else if(line[0] == 'R'){
                    std::cout << line << "\n";
                } 
                
                else {
                    uart::send("ERR unknown\n");
                }
            }
        }
    }




    std::cout << "Interrupt Detected!\n";

    int16_t end_sig = -999;
    uart::send(std::string_view(reinterpret_cast<const char*>(&end_sig), 2));
    std::string dir = uart::recv(1, 100);

    if (dir.size() == 1) {
        std::cout << "Sent h_error: " << end_sig
                    << "  MCU sent: " << dir[0] << "\n";
    } else {
        std::cout << "Nothing Received (timeout)\n";
    }

    // Turn off the lamps
    int rc = pixy.setLamp(0, 0);
    if (rc < 0) std::cerr << "setLamp failed: " << rc << "\n";
    else std::cout << "Lamps off\n";

    uart::close();
    return 0;
}





void handle_SIGINT(int unused) {
    run_flag = false;
}

void recv_float_with_timeout(uint8_t rxsize, uint32_t timeout_ms, float sent_val){
  
    std::string resp = uart::recv(rxsize, timeout_ms);
    if (resp.size() == rxsize) {
        float echo = 0.0f;
        std::memcpy(&echo, resp.data(), rxsize);
        std::cout << "Sent: " << sent_val << "  MCU echoed: " << echo << "\n";
    } else {
        std::cout << "Nothing Received (timeout)\n";
    }

}