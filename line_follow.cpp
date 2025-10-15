#include "uart.hpp"
#include "libpixyusb2.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <cstring>
#include <csignal>

#define X_CENTER (pixy.frameWidth/2) // center of the Pixy2 camera's view (316 pixels wide)


static bool run_flag = true;

void get_line_features(Pixy2 &pixy);
int get_heading_error(Pixy2 &pixy);
void handle_SIGINT(int unused);
void recv_float_with_timeout(uint8_t rxsize, uint32_t timeout_ms, float sent_val);

float kp = 8.0f;
float ki = 0.5f;
float kd = 0.2f;

// pack int16_t LE explicitly to avoid endianness surprises
static inline void pack_le16(int16_t v, uint8_t out[2]) {
    out[0] = static_cast<uint8_t>(v & 0xFF);
    out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

int main() {
    std::signal(SIGINT, handle_SIGINT);
    std::cout << "Line follow demo\n";

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
    
    /*
    uart::send("Test");
    std::string resp = uart::recv(256, 1000); // up to 256 bytes, 1s timeout
    if(resp.empty()) {
        std::cout << "no response\n";
    } else {
        std::cout << "UART active. Response: " << resp << "\n";
    }
    */

  
    
    uart::send(std::string_view(reinterpret_cast<const char*>(&kp), sizeof(kp)));
    recv_float_with_timeout(4, 1000, kp);
    uart::send(std::string_view(reinterpret_cast<const char*>(&ki), sizeof(ki)));
    recv_float_with_timeout(4, 1000, ki);
    uart::send(std::string_view(reinterpret_cast<const char*>(&kd), sizeof(kd)));
    recv_float_with_timeout(4, 1000, kd);


    // Switch to line tracking mode
    rc = pixy.changeProg("line");
    if (rc < 0) { std::cerr << "changeProg failed: " << rc << "\n";}
    else { std::cout << "Line tracking mode active\n"; }

    //Turn on the lamps
    rc = pixy.setLamp(1, 1);
    if (rc < 0) std::cerr << "setLamp failed: " << rc << "\n";
    else std::cout << "Lamps on\n";

    // Set LED to green
    rc = pixy.setLED(0, 0, 0); // green
    if (rc < 0) std::cerr << "setLED failed: " << rc << "\n";
    else std::cout << "LED green\n";



    while (run_flag) {
        int16_t h_error = get_heading_error(pixy);  // your function

        // send 2 bytes (LE)
        uint8_t tx[2];
        pack_le16(h_error, tx);
        uart::send(std::string_view(reinterpret_cast<const char*>(tx), 2));


        std::string dir = uart::recv(2, 100);
        if (dir.size() == 2) {
            std::cout << "Sent h_error: " << h_error
                      << "  Left Motor: " << static_cast<unsigned>(static_cast<uint8_t>(dir[0]))
                      << "  Right Motor: " << static_cast<unsigned>(static_cast<uint8_t>(dir[1]))
                      << "\n";
        } else {
            std::cout << "Nothing Received (timeout)\n";
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
    rc = pixy.setLamp(0, 0);
    if (rc < 0) std::cerr << "setLamp failed: " << rc << "\n";
    else std::cout << "Lamps off\n";

    uart::close();
    return 0;
}


int get_heading_error(Pixy2 &pixy){
    int32_t error;
 // center of the Pixy2 camera's view (316 pixels wide)    

    pixy.line.getMainFeatures();
    if(pixy.line.numVectors){
        //pixy.line.vectors[0].print();
        error = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;
        //std::cout << "Heading error: " << error << "\n";
        return error;
    } else {
        std::cout << "No vectors detected\n";
        return -999; 
    }

}

void  get_line_features(Pixy2 &pixy)
{
  int  Element_Index;

  // Query Pixy for line features //
  pixy.line.getAllFeatures();

  // Were vectors detected? //
  if (pixy.line.numVectors)
  {
    // Blocks detected - print them! //

    printf ("Detected %d vectors(s)\n", pixy.line.numVectors);

    for (Element_Index = 0; Element_Index < pixy.line.numVectors; ++Element_Index)
    {
      printf ("  Vector %d: ", Element_Index + 1);
      pixy.line.vectors[Element_Index].print();
    }
  }

  // Were intersections detected? //
  if (pixy.line.numIntersections)
  {
    // Intersections detected - print them! //

    printf ("Detected %d intersections(s)\n", pixy.line.numIntersections);

    for (Element_Index = 0; Element_Index < pixy.line.numIntersections; ++Element_Index)
    {
      printf ("  ");
      pixy.line.intersections[Element_Index].print();
    }
  }

  // Were barcodes detected? //
  if (pixy.line.numBarcodes)
  {
    // Barcodes detected - print them! //

    printf ("Detected %d barcodes(s)\n", pixy.line.numBarcodes);

    for (Element_Index = 0; Element_Index < pixy.line.numBarcodes; ++Element_Index)
    {
      printf ("  Barcode %d: ", Element_Index + 1);
      pixy.line.barcodes[Element_Index].print();
    }
  }
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