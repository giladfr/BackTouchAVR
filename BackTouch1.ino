
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



// Button shield defines
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5


// Debug flags
// #define TC_RAW_PRINT
//#define DELTA_PRINT

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
#define AVG_NUM_OF_POINTS 10
Point pnt_arr[AVG_NUM_OF_POINTS];
#define AVG_SMART_DELTA   5





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
  
  *pnt_dx = -dx;
  *pnt_dy = dy;

}

Point CalcMovingAvg_Simple(Point inPnt)
{
  static int i = 0;
  static int sum_x = 0;
  static int sum_y = 0;
  Point retPnt;

  // Remove from sum the point in place i
  sum_x = sum_x - pnt_arr[i].x;
  sum_y = sum_y - pnt_arr[i].y;

  // insert the current point
  pnt_arr[i] = inPnt;

  sum_x = sum_x + inPnt.x;
  sum_y = sum_y + inPnt.y;
  
  retPnt.x = sum_x / AVG_NUM_OF_POINTS;
  retPnt.y = sum_y / AVG_NUM_OF_POINTS;

  i++;
  if (i == AVG_NUM_OF_POINTS) i=0;

  return retPnt;
}

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
      break;
    case MODE_POINTER:
      lcd.print("POINTER  ");
      dy_t = 1;
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
  int pnt_dy = 0;
  int pnt_dx = 0;
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
    // if (Mouse.isPressed(1) == 0)
    // {
    //   Mouse.press(1);
    //   Serial.print("****Press****\n");
    // }

    // Serial.print("X = "); Serial.print(p.x);
    // Serial.print("\tY = "); Serial.print(p.y);
    // Serial.print("\tPressure = "); Serial.println(p.z);
    lcd.setCursor(0,1);
    lcd.print("X");
    lcd.print(p.x);
    lcd.print(" Y");
    lcd.print(p.y);
    lcd.print(" Z");
    lcd.print(p.z);
    lcd.print("  ");

    p = CalcMovingAvg_Simple(p);


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
      CalcMovment_Pointer(dx,dy,&pnt_dx,&pnt_dy);
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
    delay(sl_scrl);
    last_p = p;
  }
  else //(p.z > ts.pressureThreshhold)
  {
    // if (Mouse.isPressed(1))
    // {
    //   Mouse.release(1);
    //   Serial.print("****Release****\n");

    // }


  }

}

