
#include <stdint.h>
#include "TouchScreen.h"
#include <LiquidCrystal.h>


// #define YP A2  // must be an analog pin, use "An" notation! (Blue)
// #define XM A3  // must be an analog pin, use "An" notation! (White)
// #define YM 8   // can be a digital pin (Purple)
// #define XP 9   // can be a digital pin (Red)


// PIN defines
#define YP A3  // must be an analog pin, use "An" notation! (Blue)
#define XM A4  // must be an analog pin, use "An" notation! (White)
#define YM A2   // can be a digital pin (Purple)
#define XP A5   // can be a digital pin (Red)#define PIN_BACKLIGHT    10

#define PIN_BACKLIGHT     10



// Button shield defines
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5


// Debug flags
// #define TC_RAW_PRINT
#define DELTA_PRINT

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using.
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 330);
Point last_p;
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);




// Globals
int lcd_key     = 0;
int adc_key_in  = 0;

int dy_t = 5; // dy threshold
int dx_t = 5; // dx threshold
int sl_scrl = 5; // sleep after sending a scroll event
int sl_nscrl = 10; // sleep after not sending a scroll event







int read_LCD_buttons()
{
 adc_key_in = analogRead(0);   
 if (adc_key_in > 1000) return btnNONE; 
 if (adc_key_in < 50)   return btnRIGHT;  
 if (adc_key_in < 195)  return btnUP; 
 if (adc_key_in < 380)  return btnDOWN; 
 if (adc_key_in < 555)  return btnLEFT; 
 if (adc_key_in < 790)  return btnSELECT;    
 
 return btnNONE; 
}


int LCD_action()
{
  lcd.setCursor(0,0);
  lcd_key = read_LCD_buttons();  // read the buttons

  switch (lcd_key)               // depending on which button was pushed, we perform an action
  {
    case btnRIGHT:
    {
      sl_scrl++;
      lcd.print("sl_scrl = "); lcd.print(sl_scrl); lcd.print("      ");
      delay(200);
      break;
    }
    case btnLEFT:
    {
      sl_scrl--;
      lcd.print("sl_scrl = "); lcd.print(sl_scrl); lcd.print("      ");
      delay(200);
      break;
    }
    case btnUP:
    {
      dy_t++;
      lcd.print("dy_t = "); lcd.print(dy_t); lcd.print("      ");
      delay(200);
      break;
    }
    case btnDOWN:
    {
      dy_t--;
      lcd.print("dy_t = "); lcd.print(dy_t); lcd.print("      ");
      delay(200);
  break;
    }
    case btnSELECT:
    {
      lcd.print("SELECT");
      break;
    }
  }

  // Wait until no button is pressed
  // while(read_LCD_buttons() != btnNONE);

  // Just sleep

}



void setup(void) 
{
    // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  pinMode(PIN_BACKLIGHT,  OUTPUT);
  digitalWrite(PIN_BACKLIGHT, HIGH);
  
  // Write welcome messege
  lcd.print("BackTouch V0.1");
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


// #ifdef DEBUG_TOUCH_SCREEN
  // Serial.print("X = "); Serial.print(p.x);
  // Serial.print("\tY = "); Serial.print(p.y);
  // Serial.print("\tPressure = "); Serial.println(p.z);
  // delay(100);
// #endif

  LCD_action();
  
  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!
  if (p.z > ts.pressureThreshhold) 
  {

    // Serial.print("X = "); Serial.print(p.x);
    // Serial.print("\tY = "); Serial.print(p.y);
    // Serial.print("\tPressure = "); Serial.println(p.z);
    lcd.setCursor(0,1); lcd.print("X");
    lcd.print(p.x); lcd.print(" Y");
    lcd.print(p.y); lcd.print(" Z");
    lcd.print(p.z); lcd.print("  ");


    dx = last_p.x - p.x;
    dy = last_p.y - p.y;

#ifdef DELTA_PRINT
    Serial.print("DX = "); Serial.print(dx);
    Serial.print("\tDY = "); Serial.println(dy); 
#endif

    if (abs(dy) > dy_t)
    {
      if (dy>0) // positive dy
      {
        vr_scrl = 1; 
      }
      else
      { 
        vr_scrl = -1; // up scroll should be more responsive maybe
      }
    }

    if (abs(dx) > dx_t)
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



    if ((vr_scrl != 0) || (hz_scrl != 0)) // passed the threshold send scroll event
    {
      Serial.print("vr_scrl = "); Serial.print(vr_scrl);
      Serial.print("\thz_scrl = "); Serial.println(hz_scrl);
      Mouse.move(0, 0, -vr_scrl, hz_scrl); // reverse scrolling on android
      delay(sl_scrl);
    }
    else // if we're not sending scroll event, just wait 5ms for next sample to be more precise
    {
      delay(sl_nscrl);
    }
    
    last_p = p;

  }

}