
#include <stdint.h>
#include "TouchScreen.h"

// #define YP A2  // must be an analog pin, use "An" notation! (Blue)
// #define XM A3  // must be an analog pin, use "An" notation! (White)
// #define YM 8   // can be a digital pin (Purple)
// #define XP 9   // can be a digital pin (Red)


#define YP A3  // must be an analog pin, use "An" notation! (Blue)
#define XM A4  // must be an analog pin, use "An" notation! (White)
#define YM A2   // can be a digital pin (Purple)
#define XP A5   // can be a digital pin (Red)



// #define TC_RAW_PRINT
#define DELTA_PRINT

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using.
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 330);
Point last_p;


void setup(void) 
{
  Serial.begin(9600);
  Mouse.begin();

}

void loop(void) 
{
  int dy = 0;
  int dx = 0;

  int hz_scrl = 0;
  int vr_scrl = 0;
  // a point object holds x y and z coordinates
  Point p = ts.getPoint();


#ifdef DEBUG_TOUCH_SCREEN
  Serial.print("X = "); Serial.print(p.x);
  Serial.print("\tY = "); Serial.print(p.y);
  Serial.print("\tPressure = "); Serial.println(p.z);
  delay(100);
#endif



  
  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!
  if (p.z > ts.pressureThreshhold) 
  {

#ifdef TC_RAW_PRINT
     Serial.print("X = "); Serial.print(p.x);
     Serial.print("\tY = "); Serial.print(p.y);
     Serial.print("\tPressure = "); Serial.println(p.z);
#endif


    dx = last_p.x - p.x;
    dy = last_p.y - p.y;

#ifdef DELTA_PRINT
    Serial.print("DX = "); Serial.print(dx);
    Serial.print("\tDY = "); Serial.println(dy); 
#endif

    if (abs(dy) > 5)
    {
      if (dy>0) // positive dy
      {
        vr_scrl = 2; 
      }
      else
      { 
        vr_scrl = -2; // up scroll should be more responsive don't know what
      }
    }

    if (abs(dx) > 5)
    {
      if (dx>0) // positive dx
      {
        hz_scrl = 1; // move only one step
      }
      else
      {
        hz_scrl = -1;
      }
    }



    if ((vr_scrl != 0) || (hz_scrl != 0))
    {
      Serial.print("vr_scrl = "); Serial.print(vr_scrl);
      Serial.print("\thz_scrl = "); Serial.println(hz_scrl);
      Mouse.move(0, 0, -vr_scrl, hz_scrl); // reverse scrolling on android
      delay(5);
    }
    else // if we're not sending scroll even, just wait 5ms for next sample to be more precise
    {
      delay(10);
    }
    
    last_p = p;

  }

}