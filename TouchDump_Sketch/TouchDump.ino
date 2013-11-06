
#include <stdint.h>
#include "TouchScreen.h"
#include <LiquidCrystal.h>


// #define YP A2  // must be an analog pin, use "An" notation! (Blue)
// #define XM A3  // must be an analog pin, use "An" notation! (White)
// #define YM 8   // can be a digital pin (Purple)
// #define XP 9   // can be a digital pin (Red)


// PIN defines
#define YP A5  // must be an analog pin, use "An" notation! (Blue)
#define XM A2  // must be an analog pin, use "An" notation! (White)
#define YM A4   // can be a digital pin (Purple)
#define XP A3   // can be a digital pin (Red)#define PIN_BACKLIGHT    10

#define PIN_BACKLIGHT     10


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
  // a point object holds x y and z coordinates
  Point p = ts.getPoint();


  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!
  if (p.z > ts.pressureThreshhold)
  {
    Serial.print(p.x); Serial.print(","); 
		Serial.print(p.y); Serial.print(","); 
		Serial.println(p.z);
	}
}

