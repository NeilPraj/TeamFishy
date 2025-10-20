#include "camera.hpp"



#ifndef X_CENTER
#define X_CENTER 39
#endif



int get_heading_error(Pixy2 &pixy)
{
    pixy.line.getMainFeatures();
    if (pixy.line.numVectors == 0) {
        std::cout << "No vectors detected\n";
        return -999;  // or -999 sentinel
    }

    const auto& v = pixy.line.vectors[0];
    int x_shift = v.m_x1 - X_CENTER;  // horizontal offset in pixels
    //std::cout << "x_shift: " << x_shift << "\n";
    return x_shift;
}





int get_theta_deg(Pixy2& pixy)
{
    // Always refresh line features first
    int rc = pixy.line.getMainFeatures();
    if (rc < 0) {
        std::cerr << "getMainFeatures() failed: " << rc << "\n";
        return -999;
    }
    if (pixy.line.numVectors <= 0) {
        return -999; // no line this frame
    }

    const auto& v = pixy.line.vectors[0];

    // Pixy coordinate system: x→ right, y→ down
    const float dx = float(v.m_x1 - v.m_x0);
    const float dy = float(v.m_y1 - v.m_y0);

    // Angle relative to "forward" (downwards): 0° when vertical/straight ahead
    const float theta_rad = std::atan2(dx, dy);

    // C++14-safe PI and conversion
    constexpr float PI = 3.14159265358979323846f;
    const float theta_deg_f = theta_rad * 180.0f / PI;

    if (!std::isfinite(theta_deg_f)) return -999;
    return int(std::lrint(theta_deg_f));
}

/*
int get_heading_error(Pixy2 &pixy){
    int32_t error;
 // center of the Pixy2 camera's view (316 pixels wide)    

    pixy.line.getMainFeatures();
    
    if(pixy.line.numVectors){
        //printf("%u\n", pixy.line.numVectors);
        //pixy.line.vectors[0].print();
        int32_t opp = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;   // Horizontal offset
        int32_t adj = (int32_t)pixy.line.vectors->m_y1;     // Uppermost y component
        float theta = atan((float)opp / (float)adj) * (180 / M_PI); // Offset angle
        error = (int32_t)theta;
        // std::cout << "Heading error: " << error << " degrees\n";
        return error;
    } else {
        std::cout << "No vectors detected\n";
        return -999; 
    }
}
*/


void  get_line_features(Pixy2 &pixy){
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

int setup_camera(Pixy2 &pixy){
    int rc = pixy.init();
    if (rc < 0) {
        std::cerr << "pixy init failed: " << rc << "\n";
        return rc;
    }

    std::cout << "FW: "
            << (int)pixy.version->firmwareMajor << "."
            << (int)pixy.version->firmwareMinor << "."
            << (int)pixy.version->firmwareBuild << "\n";

    rc = pixy.changeProg("line");
    if (rc < 0) {
        std::cerr << "changeProg(\"line\") failed: " << rc << "\n";
        return rc; 
    }
    std::cout << "Line tracking mode active\n";

    rc = pixy.setLamp(1, 1);
    if (rc < 0) {
        std::cerr << "setLamp failed: " << rc << "\n";
        return rc;
    }
    std::cout << "Lamps on\n";

    // optional set LED to green to indicate successful init
    rc = pixy.setLED(0, 255, 0);
    if (rc < 0) {
        std::cerr << "setLED failed: " << rc << "\n";
        return rc;
    }
    std::cout << "LED green\n";

    return 0; // <-- required
}
