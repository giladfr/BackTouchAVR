#include "Arduino.h"
#include <stdint.h>
#include "TouchScreen.h"
#include <LiquidCrystal.h>
#include <Wire.h>


// #define YP A2  // must be an analog pin, use "An" notation! (Blue)
// #define XM A3  // must be an analog pin, use "An" notation! (White)
// #define YM 8   // can be a digital pin (Purple)
// #define XP 9   // can be a digital pin (Red)

 #define PLATFORM_MICRO
// #define PLATFORM_LEONARDO

// PIN defines - Blue 
#ifdef PLATFORM_LEONARDO
#define YP A5  // must be an analog pin, use "An" notation! (Blue)
#define XM A2  // must be an analog pin, use "An" notation! (White)
#define YM A4   // can be a digital pin (Purple)
#define XP A3   // can be a digital pin (Red)#define PIN_BACKLIGHT    10
#endif

// PIN Defines - Pro Micro
#ifdef PLATFORM_MICRO
#define YP A3  // must be an analog pin, use "An" notation! (Blue)
#define XM A0  // must be an analog pin, use "An" notation! (White)
#define YM A2   // can be a digital pin (Purple)
#define XP A1   // can be a digital pin (Red)#define PIN_BACKLIGHT    10
#endif


#define PIN_BACKLIGHT     10
#define PIN_TS_INTERRUPT  0
#define PIN_TS_RST        1


// Button shield defines
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5


// Debug flags
// #define TC_RAW_PRINT
// #define DELTA_PRINT
#define SEND_CLICK

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

typedef enum
{
  MODE_POINTER,
  MODE_SCROLL,
  MODE_LAST,
} OperationModes;
int cur_mode = MODE_POINTER; // default mode


// Average array
#define AVG_NUM_OF_POINTS 5
// Point pnt_arr[AVG_NUM_OF_POINTS];
#define AVG_SMART_DELTA   5
int x_arr[AVG_NUM_OF_POINTS];
int y_arr[AVG_NUM_OF_POINTS];
int z_arr[AVG_NUM_OF_POINTS];

#define RELEASE_THRESHOLD_MILIS 100
bool isFingerDown = false;
int last_touch_time = 0;





void CalcMovment_Scroll(int dx, int dy, int* hz_scrl, int *vr_scrl)
{
  if (abs(dy) > dy_t)
  {
    if (dy>0) // positive dy
    {
      *vr_scrl = 1;
    }
    else
    {
      *vr_scrl = -1; // up scroll should be more responsive maybe
    }
  }

  if (abs(dx) > dx_t)
  {
    if (dx>0) // positive dx
    {
      *hz_scrl = 1; // move only one step
    }
    else
    {
      *hz_scrl = -1;
    }
  }
}


void CalcMovment_Pointer(int dx, int dy,int* pnt_dx,int *pnt_dy)
{
/*  if (abs(dy) > dy_t)
  {
    if (dy > 0)
    {
      *pnt_dy = 5;

    }
    else
    {

      *pnt_dy = -5;

    }
  }

  if (abs(dx) > dy_t)
  {
    *pnt_dx = dx;
  }
  *pnt_dx = 0;*/
  
  *pnt_dx = dx;
  *pnt_dy = dy;

}

Point CalcMovingAvg_Simple(Point inPnt)
{
  static int i = 0;
  int sum_x = 0;
  int sum_y = 0;
  int n_not_zero = 0;
  Point retPnt;

  // insert the current point
  x_arr[i] = inPnt.x;
  y_arr[i] = inPnt.y;

  // Calc how many slots are not zero and the sum of them
  for(int j = 0 ; j < AVG_NUM_OF_POINTS ; j++)
  {
    if (x_arr[j] != 0)
    {
      n_not_zero++;
      sum_x = sum_x + x_arr[j];
      sum_y = sum_y + y_arr[j];
    } 
  }
  
  retPnt.x = sum_x / n_not_zero;
  retPnt.y = sum_y / n_not_zero;
  retPnt.z = inPnt.z;

  if (++i == AVG_NUM_OF_POINTS) i=0;

  return retPnt;
}

/*
// Assume movment is fluid and get rid of points more then a delta away
Point CalcMovingAvg_Smart(Point inPnt)
{
  static int i = 0;
  // static int last_x_avg;
  static Point last_ret_pnt;
  Point retPnt;


  // insert the current instead of old point
  pnt_arr[i] = inPnt;

  // Calc Average for only the points that are closer then delta
  int npnt_x = 0;
  int npnt_y = 0;
  int sum_x = 0;
  int sum_y = 0;
  Serial.print("i = "); Serial.println(i);
  for (int j = 0;j < AVG_NUM_OF_POINTS; j++)
  {
    if (abs(pnt_arr[j].x - last_ret_pnt.x) < AVG_SMART_DELTA)
    {
      sum_x = sum_x + pnt_arr[j].x;
      npnt_x++;
    }
    if (abs(pnt_arr[j].y - last_ret_pnt.y) < AVG_SMART_DELTA)
    {
      sum_y = sum_y + pnt_arr[j].y;
      npnt_y++;
    }
  }
  
  Serial.print("sum_x = "); Serial.println(sum_x);
  retPnt.x = sum_x / npnt_x;
  retPnt.y = sum_y / npnt_y;

  // Save the last returned point for filtering next time
  last_ret_pnt = retPnt;


  i++;
  if (i == AVG_NUM_OF_POINTS) i=0;

  return retPnt;
}
*/




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
    lcd.print("sl_scrl = ");
    lcd.print(sl_scrl);
    lcd.print("      ");
    delay(200);
    break;
  }
  case btnLEFT:
  {
    sl_scrl--;
    lcd.print("sl_scrl = ");
    lcd.print(sl_scrl);
    lcd.print("      ");
    delay(200);
    break;
  }
  case btnUP:
  {
    dy_t++;
    lcd.print("dy_t = ");
    lcd.print(dy_t);
    lcd.print("      ");
    delay(200);
    break;
  }
  case btnDOWN:
  {
    dy_t--;
    lcd.print("dy_t = ");
    lcd.print(dy_t);
    lcd.print("      ");
    delay(200);
    break;
  }
  case btnSELECT:
  {
    // switch operation mode
    cur_mode++;
    if (cur_mode == MODE_LAST) cur_mode = 0; // wrap around
    lcd.print("MODE: ");
    switch (cur_mode) {
    case MODE_SCROLL:
      lcd.print("SCROLL   ");
      dy_t = 5;
      sl_scrl = 5;
      break;
    case MODE_POINTER:
      lcd.print("POINTER  ");
      dy_t = 1;
      sl_scrl = 0;
      break;
    default:
      break;
      // do something
    };
    delay(500);
    break;
  }
  }

}



void setup(void)
{
#ifdef PLATFORM_LEONARDO
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  pinMode(PIN_BACKLIGHT,  OUTPUT);
  digitalWrite(PIN_BACKLIGHT, HIGH);

  // Write welcome messege
  lcd.print("BackTouch V0.2");
  Serial.begin(9600);
#endif
  Wire.begin();        // join i2c bus (address optional for master)

  Mouse.begin();


  pinMode(PIN_TS_INTERRUPT,INPUT);
  pinMode(PIN_TS_RST,OUTPUT);
  digitalWrite(PIN_TS_RST,HIGH);
  delay(100);
  digitalWrite(PIN_TS_RST,LOW);

}




volatile char rawTouchPacket[6]; 

void ReadTouchPacket()
{

  Wire.requestFrom(0x0A, 6);    // request 6 bytes from slave device #2
  int i = 0;

  while(Wire.available())    // slave may send less than requested
  { 
    rawTouchPacket[i++] = Wire.read(); // receive a byte as character
  }

}

void ParseTouchPacket(int16_t *x, int16_t *y, int16_t *z,uint8_t *status)
{
  *status = rawTouchPacket[0] & 1;
   *x = (rawTouchPacket[1]<<7) | rawTouchPacket[2];
   *y = (rawTouchPacket[3]<<7) | rawTouchPacket[4];
   *z = rawTouchPacket[5];
}



void loop(void)
{
  int dy = 0;
  int dx = 0;

  int hz_scrl = 0;
  int vr_scrl = 0;
  int pnt_dy = 0;
  int pnt_dx = 0;
  // a point object holds x y and z coordinates


  // Point p = ts.getPoint();
  Point p;
  uint8_t s;
  


  int pinVal;

  while(digitalRead(PIN_TS_INTERRUPT) == 1)
  {
    if (isFingerDown == true)
    {
      int cur_time = millis();
      if (cur_time - last_touch_time > RELEASE_THRESHOLD_MILIS)
      {
#ifdef SEND_CLICK
        if (cur_mode == MODE_POINTER) Mouse.release(7);
#endif
#ifdef DELTA_PRINT
        Serial.println("****Release****");
#endif
        isFingerDown = false;
        // Clear all the static arrays and the last point       
        memset(&x_arr,0,sizeof(int)*AVG_NUM_OF_POINTS);
        memset(&y_arr,0,sizeof(int)*AVG_NUM_OF_POINTS);
        memset(&z_arr,0,sizeof(int)*AVG_NUM_OF_POINTS);
        memset(&last_p,0,sizeof(last_p));

      }
    }

  };

  ReadTouchPacket();
  ParseTouchPacket(&p.x,&p.y,&p.z,&s);
  // Serial.println(s);


#ifdef PLATFORM_LEONARDO
  LCD_action();
#endif

  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!
  if ((s == 1))
  {

#ifdef PLATFORM_LEONARDO
    lcd.setCursor(0,1);
    lcd.print("X");
    lcd.print(p.x);
    lcd.print(" Y");
    lcd.print(p.y);
    lcd.print(" Z");
    lcd.print(p.z);
    lcd.print("    ");
#endif
    
    // p = CalcMovingAvg_Simple(p);

    dx = last_p.x - p.x;
    dy = last_p.y - p.y;


#ifdef DELTA_PRINT
    Serial.print("DX = ");
    Serial.print(dx);
    Serial.print("\tDY = ");
    Serial.println(dy);
#endif

    if (cur_mode == MODE_SCROLL) // Do thresholding on scroll mode
    {
      CalcMovment_Scroll(dx,dy,&hz_scrl,&vr_scrl);
      pnt_dx = 0;
      pnt_dy = 0;
    }
    else // POINTER MODE
    {
      //CalcMovment_Pointer(dx,dy,&pnt_dx,&pnt_dy);
      pnt_dx = 1024 - (p.y>>1);//(uint16_t)(-1.46 * (float)p.x + 1256);
      pnt_dy = p.x>>1;//(uint16_t)(-1.27 * (float)p.y + 1150);
      hz_scrl = 0;
      vr_scrl = 0;
    }

    if ((vr_scrl != 0) || (hz_scrl != 0) || (pnt_dy != 0) || (pnt_dx != 0)) // passed the threshold send scroll / move event
    {
      Mouse.move(pnt_dx, pnt_dy, -vr_scrl, hz_scrl); // reverse scrolling on android
#ifdef DELTA_PRINT
      Serial.print("vr_scrl = ");
      Serial.print(vr_scrl);
      Serial.print("\thz_scrl = ");
      Serial.print(hz_scrl);
      Serial.print("\tpnt_dx = ");
      Serial.print(pnt_dx);
      Serial.print("\tpnt_dy = ");
      Serial.println(pnt_dy);
#endif
    }

    if (isFingerDown == false)
    {
#ifdef SEND_CLICK
      if (cur_mode == MODE_POINTER) 
      {
        Mouse.press(7);
        // delay(1);
        // Mouse.release(7);
        // delay(1);
        // Mouse.press(7);
      }
#endif
#ifdef DELTA_PRINT
       Serial.println("****Press****");
#endif
       isFingerDown = true;
       // last_p = p; // Last point is the same as this point so cursor starts where you left it. 
    }

    // delay(sl_scrl);
    last_p = p;

    last_touch_time = millis();
  }
  else //(p.z > ts.pressureThreshhold)
  {
  }

}

