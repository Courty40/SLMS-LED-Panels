/******************************************************************************
This is an Arduino sketch to drive 4 LED panels based on MBI5034 LED drivers.

Written by Oliver Dewdney and Jon Russell

It works specifically on a Arduino Micro (ATMega32U4), as it accesses the 
registers directly. However, with a small amount of editing, it should work on 
most Arduinos.

Basic Operation:

The objective was to be able to drive four panels with a single Arduino. 
Each panel has two data lines, so four panels require 8 data lines, i.e. a 
single byte.

An Arduino Micro was selected, as it has a native USB port and slightly more
RAM than an Uno. A Mega would probably work well too.

The code has a frame buffer in RAM with 4 sets of 384 bits 
(1 bank = 64 LEDs x 2 Rows x 3 bits (RGB) = 384) for each data line. 
Four panels, two data lines each, means all four panels can be driven by a byte 
wide frame buffer, assuming 3 bit colour. This means the update can be very 
efficient. The frame buffer is about 1.5KB so fits happily in to the ATMega32U4
with some room for local variables.

The UpdateFrame loop iterates over the line of 384 bits of serial data from the
frame buffer and clocks them out quickly.

Ideally, we needed a contiguous port on the Microcontroller to be attached to 
8 external data lines for the four panels. But most Arduinos don’t have this. 
On the Arduino Micro, PortB has the RX LED on bit 0 and PortD has the TX LED on
bit 5. So, I have connected 4 data lines to the high nibble on PortB and 4 data
lines to the low nibble on PortD.

If we had a contiguous port free (If we used a Mega) we could use only one port
and the UpdateFrame loop would be 1 instruction faster ! :-) But currently I 
copy the data to both ports, ignoring the high and low nibbles I don’t need.

UpdateFrame is called by an interrupt 390 times a second 
(OCR1A = 160; // compare match register 16MHz/256/390Hz)). 
UpdateFrame needs to be called 4 times to update the entire panel, as the panel
is split in to 4 rows of 128 LEDs (as 2 rows of 64).

For each half a panel (one data line) there are 8 rows of 64 LEDs, addresses in 
banks. Bank 0x00 is row 0&4,  Bank 0x01 is row 1&5, Bank 0x02 is row 2&6, Bank 
0x03 is row 3&7.

Each call updates one bank and leaves it lit until the next interrupt.

This method updates the entire frame (1024 RGB LEDs) about 100 times a second.

Port map for Arduino Micro (ATMega32U4)
    7     6     5     4     3     2     1     0
PB  D11   D10   D9    D8    MISO  MOSI  SCK   RX/SS
PC  D13   D5    X     X     X     X     X     X
PD  D6    D12   TX    D4    D1    D0    D2    D3
PE  X     D7    X     X     X     HWB   X     X
PF  A0    A1    A2    A3    X     X     A4    A5

This sketch now inherits from the Adafruit GFX library classes. This means we
can use the graphics functions like drawCircle, drawRect, drawTriangle, plus
the various text functions and fonts.

******************************************************************************

Updated by Courty 21/07/17
fixed drawBitmap calls
added bmp.h and seperated display pages in main for rolling display
added 'macro to text commands' to store chars in Filemem and clean up somespace in RAM

******************************************************************************/

#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <fonts/FreeSerifBoldItalic9pt7b.h>
#include "bmp.h"
#include "digitalWriteFast.h"
#include "Ticker.h"

class LedPanel : public Adafruit_GFX
{
  public:
    LedPanel() : Adafruit_GFX(64,64) {};
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    uint16_t newColor(uint8_t red, uint8_t green, uint8_t blue);
    uint16_t getColor() { return textcolor; }
    void drawBitmapMem(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
};

LedPanel panel;
Ticker timer1;

#define PIN_D1    3   //PD0 - D1 on Panel 1
#define PIN_D2    2   //PD1 - D2 on Panel 1
#define PIN_D3    0   //PD2 - D1 on Panel 2
#define PIN_D4    1   //PD3 - D2 on Panel 2

#define PIN_D5    8   //PB4 - D1 on Panel 3
#define PIN_D6    9   //PB5 - D2 on Panel 3
#define PIN_D7    10  //PB6 - D1 on Panel 4
#define PIN_D8    11  //PB7 - D2 on Panel 4

#define PIN_A0    A0  //PF1 - A0 on all Panels
#define PIN_A1    A1  //PF6 - A1 on all Panels
#define PIN_CLK   A3  //PF4 - CLK on all Panels
#define PIN_LAT   A2  //PF5 - LAT on all Panels
#define PIN_OE    15  //PF0 - OE on all Panels

#define LED_BLACK 0
#define LED_BLUE 1
#define LED_GREEN 2
#define LED_CYAN 3
#define LED_RED 4
#define LED_MAGENTA 5
#define LED_YELLOW 6
#define LED_WHITE 7

byte frame[4][384];

void FillBuffer(byte b){
  for(uint8_t x=0; x<4; x++){
    for(uint16_t y=0; y<384; y++){
      frame[x][y] = b;
    }
  }
}

void LedPanel::drawPixel(int16_t x, int16_t y, uint16_t color) {
  setpixel(x,y,color);
}

uint16_t LedPanel::newColor(uint8_t red, uint8_t green, uint8_t blue) {
  return (blue>>7) | ((green&0x80)>>6) | ((red&0x80)>>5);
}

void LedPanel::drawBitmapMem(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {
  int16_t i, j, byteWidth = (w + 7) / 8;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(bitmap[j * byteWidth + i / 8] & (128 >> (i & 7))) {
        panel.drawPixel(x+i, y+j, color);
      }
    }
  }
}

// bb describes which data lines drive which of the 4 panels.
// By adjusting the order of the bits in the array, you can change the panel order visually.
byte bb[8] = { 0x40, 0x80, 0x10, 0x20, 0x04, 0x08, 0x01, 0x02 };

// Set a pixel to a specific 3 bit colour (8 colours)
// 0b000 = black (off), 0b001 = Blue, 0b010 = Green, 0b100 = Red, 0b011 = Cyan, 0b101 = Magenta, 0b110 = Yellow, 0b111 = White, etc.
void setpixel(byte x, byte y, byte col) {
  int16_t off = (x&7) + (x & 0xf8)*6 + ((y & 4)*2);
//  int16_t off = (x&7) + (x >> 3)*48 + ((y & 4)*2);
  byte row = y & 3;
  byte b = bb[(y&0x3f) >> 3];
  byte *p = & frame[row][off];
  for(byte c = 0; c < 3;c++) {
    if ( col & 1 ) {
      *p |= b;
    } else {
      *p &= ~b;
    }
    col >>= 1;
    p += 16;
  }
}

uint8_t bank = 0;

void UpdateFrame() {
  byte * f = frame[bank];
  for (uint16_t n = 0; n<384; n++) {
    PORTD = *f;      // We use the low nibble on PortD for Panel 1 & 2
    PORTB = *f++;    // We use the high nibble on PortB for Panel 3 & 4
    digitalWriteFast(PIN_CLK, LOW);
    digitalWriteFast(PIN_CLK, HIGH);
    }

  digitalWriteFast(PIN_OE,HIGH);     // disable output
  if (bank & 0x01) {
    digitalWriteFast(PIN_A0, HIGH);
  } else {
    digitalWriteFast(PIN_A0, LOW);
  }
  if (bank & 0x02) {
    digitalWriteFast(PIN_A1, HIGH);
  } else {
    digitalWriteFast(PIN_A1, LOW);
  }
  digitalWriteFast(PIN_LAT, HIGH);   // toggle latch
  digitalWriteFast(PIN_LAT, LOW);
  digitalWriteFast(PIN_OE, LOW);     // enable output

  if (++bank>3) bank=0;
}


void setup() {
  Serial.begin(115200);
  //Serial.println("Begin...");
  
  pinMode(PIN_D1, OUTPUT);
  pinMode(PIN_D2, OUTPUT);
  pinMode(PIN_D3, OUTPUT);
  pinMode(PIN_D4, OUTPUT);

  pinMode(PIN_D5, OUTPUT);
  pinMode(PIN_D6, OUTPUT);
  pinMode(PIN_D7, OUTPUT);
  pinMode(PIN_D8, OUTPUT);

  pinMode(PIN_A0, OUTPUT);
  pinMode(PIN_A1, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_LAT, OUTPUT);
  pinMode(PIN_OE, OUTPUT);

  digitalWrite(PIN_D1, LOW);
  digitalWrite(PIN_D2, LOW);
  digitalWrite(PIN_D3, LOW);
  digitalWrite(PIN_D4, LOW);

  digitalWrite(PIN_A0, LOW);
  digitalWrite(PIN_A1, LOW);

  digitalWrite(PIN_OE, HIGH);
  digitalWrite(PIN_LAT, LOW);
  digitalWrite(PIN_CLK, LOW);

//  FillBuffer(0xFF);         // Set all LEDs on. (White)
  FillBuffer(0x00);         // Set all LEDs off. (Black)

/*
  // initialize Timer1 ~400Hz
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 160;              // compare match register 16MHz/256/390Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
  delay(100);

*/

  FillBuffer(0x00);
  panel.setTextSize(1);
  panel.setCursor(15, 17);
  panel.setTextColor(LED_WHITE);  
  panel.println(F("COURTY"));
  panel.setCursor(8, 28);
  panel.setTextColor(LED_MAGENTA);  
  panel.println(F("(C) 2017"));

  delay(1000);
  panel.fillScreen(LED_WHITE);
  delay(5000);

  timer1.setCallback(UpdateFrame);
  timer1.setInterval(100);
  timer1.start(); 

}

/*
ISR(TIMER1_COMPA_vect){     // timer compare interrupt service routine
  UpdateFrame();
}
*/

/*
void testText1() {
  FillBuffer(0x00);
  panel.setCursor(0, 0);
  panel.setTextColor(LED_GREEN);  
  panel.println("The Quick ");
  panel.setTextColor(LED_CYAN);  
  panel.println("Brown Fox ");
  panel.setTextColor(LED_RED);  
  panel.println("Jumped    ");
  panel.setTextColor(LED_MAGENTA);  
  panel.println("Over the  ");
  panel.setTextColor(LED_RED);  
  panel.println(" South Ldn");
  panel.setTextColor(LED_WHITE);  
  panel.println("MakerSpace");

}


void testText2() {
  FillBuffer(0x00);
  panel.setTextSize(1);
  panel.setCursor(15, 17);
  panel.setTextColor(LED_WHITE);  
  panel.println(F("COURTY"));
  panel.setCursor(8, 28);
  panel.setTextColor(LED_MAGENTA);  
  panel.println(F("(C) 2017"));

}



void testText3() {
  FillBuffer(0x00);
  panel.setCursor(3, 22);
  panel.setTextColor(LED_YELLOW);  
  panel.setTextSize(2);
  panel.println(F("Hello"));
}


  panel.setTextColor(LED_GREEN);  
  panel.println("The Quick ");
  panel.setTextColor(LED_CYAN);  
  panel.println("Brown Fox ");
  panel.setTextColor(LED_RED);  
  panel.println("Jumped    ");
  panel.setTextColor(LED_MAGENTA);  
  panel.println("Over the  ");
  panel.setTextColor(LED_RED);  
  panel.println(" South Ldn");
  panel.setTextColor(LED_WHITE);  
  panel.println("MakerSpace");
}


void testText() {
  panel.fillScreen(LED_BLACK);
  panel.setCursor(0, 0);
  panel.setTextColor(LED_WHITE);  
  panel.setTextSize(1);
  panel.println("Hello!");
  panel.setTextColor(LED_YELLOW); 
  panel.setTextSize(1);
  panel.println(1234.56);
  panel.setTextColor(LED_RED);    
  panel.setTextSize(1);
  panel.println(0xDEADBEEF, HEX);
  panel.setTextColor(LED_GREEN);
  panel.setTextSize(1);
  panel.println("Groop");
  panel.println("I implore thee...");
}
*/
void testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = panel.width(), h = panel.height();

  panel.fillScreen(LED_BLACK);
  for(y=0; y<h; y+=4) panel.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=4) panel.drawFastVLine(x, 0, h, color2);
}

void testFilledRects(uint16_t color1, uint16_t color2) {
  int n, i, i2,
    cx = panel.width()  / 2 - 1,
    cy = panel.height() / 2 - 1;

  panel.fillScreen(LED_BLACK);
  n = min(panel.width(), panel.height());
  for(i=n; i>0; i-=6) {
    i2 = i / 2;
    panel.fillRect(cx-i2, cy-i2, i, i, color1);
    panel.drawRect(cx-i2, cy-i2, i, i, color2);
  }
}

/*
void testFilledCircles(uint8_t radius, uint16_t color) {
  int x, y, 
    w = panel.width(), 
    h = panel.height(), 
    r2 = radius * 2;

  panel.fillScreen(LED_BLACK);
  for(x=radius; x<w; x+=r2) {
    for(y=radius; y<h; y+=r2) {
      panel.fillCircle(x, y, radius, color);
    }
  }
}

void testTriangles() {
  int n, i, 
    cx = panel.width()  / 2 - 1,
    cy = panel.height() / 2 - 1;

  panel.fillScreen(LED_BLACK);
  n = min(cx, cy);
  for(i=0; i<n; i+=4) {
    panel.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      LED_RED);
  }
}

void testRoundRects() {
  int w, i, i2,
    cx = panel.width()  / 2 - 1,
    cy = panel.height() / 2 - 1;

  panel.fillScreen(LED_BLACK);
  w = min(panel.width(), panel.height());
  for(i=0; i<w; i+=4) {
    i2 = i / 2;
    panel.drawRoundRect(cx-i2, cy-i2, i, i, i/8, LED_YELLOW);
  }
}

*/


void testFonts(){
  panel.setFont(&FreeSerifBoldItalic9pt7b);
  panel.setTextColor(LED_GREEN);  
  panel.setTextSize(0);
  panel.setCursor(0, 32);
  panel.println("Hello");
  delay(2000);
  panel.setFont();

}


void loop(){
      FillBuffer(0x00);
      panel.setCursor(3, 18);
      panel.setTextColor(LED_WHITE);  
      panel.setTextSize(2);
      panel.println(F("Hello"));
      delay(1500);
    for (int ct=0; ct <= 20; ct++){
      panel.setCursor(3, 18);
      panel.setTextColor(random(1, 7));  
      panel.setTextSize(2);
      panel.println(F("Hello"));
      delay(100);
    }
      panel.setCursor(3, 18);
      panel.setTextColor(LED_WHITE);  
      panel.setTextSize(2);
      panel.println(F("Hello"));
      delay(3000); 


      FillBuffer(0x00);
      panel.setTextSize(1);
 
      panel.setCursor(24, 5);
      panel.setTextColor(LED_YELLOW);      
      panel.println(F("And"));
      delay(300); 
      panel.setTextColor(LED_GREEN);       
      panel.setCursor(12, 20);
      panel.println(F("Welcome"));
      delay(300);
       panel.setTextColor(LED_CYAN);      
      panel.setCursor(24, 35);
      panel.println(F("too"));      
      delay(3000); 

   for (int ct=0; ct <= 44; ct++){
      FillBuffer(0x00);
      panel.setTextSize(1);
      panel.setCursor(0, ct);
      panel.setTextColor(LED_WHITE);  
      panel.println(F(" South Ldn"));
      panel.setCursor(2, ct+9);
      panel.setTextColor(LED_WHITE);  
      panel.println(F("MakerSpace"));
      delay(50);
   } 
   
      panel.drawBitmap(16, 6, (const uint8_t *)&slms32x32, 32, 32, LED_RED, LED_BLACK);
      delay(6000);

      panel.drawBitmap(0, 0, (const uint8_t *)&paul64, 64, 64, LED_GREEN, LED_BLACK);
      delay(5000);

      FillBuffer(0x00);
      panel.setTextSize(1);
      panel.setCursor(30, 5);
      panel.setTextColor(LED_YELLOW);      
      panel.println(F("A"));
      delay(300); 
      panel.setTextColor(LED_GREEN);       
      panel.setCursor(6, 20);
      panel.println(F("Community"));
      delay(300);
      panel.setTextColor(LED_MAGENTA);      
      panel.setCursor(19, 35);
      panel.println(F("Owned"));

      delay(300);
      panel.setTextColor(LED_CYAN);      
      panel.setCursor(6, 50);
      panel.println(F("Workspace"));      
      delay(3000); 


      FillBuffer(0x00);
      panel.setTextSize(1);
      panel.setCursor(2, 5);
      panel.setTextColor(LED_WHITE);      
      panel.println(F("Electronic"));
      delay(300); 
      panel.setTextColor(LED_YELLOW);       
      panel.setCursor(6, 20);
      panel.println(F("Wood Work"));
      delay(300);
      panel.setTextColor(LED_MAGENTA);      
      panel.setCursor(8, 35);
      panel.println(F("Textiles"));
      delay(300);
      panel.setTextColor(LED_GREEN);      
      panel.setCursor(8, 50);
      panel.println(F("Printing"));      
      delay(5000); 

      FillBuffer(0x00);
      panel.setTextSize(2);
      
      panel.setCursor(10, 8);
      panel.setTextColor(LED_MAGENTA);  
      panel.println(F("Join"));
      panel.setCursor(3, 38);
      panel.setTextColor(LED_CYAN);  
      panel.println(F("Today"));

      
      delay(8000); 

int ledState;
for (int ct=0; ct <= 11; ct++){
    if (ledState == LOW) {
        FillBuffer(0x00);
        panel.drawBitmap(ct*2, 20, (const uint8_t *)&Invader1, 24, 18, LED_BLACK, LED_GREEN);
        delay(300); 
      ledState = HIGH;
    } else {
        FillBuffer(0x00);
        panel.drawBitmap(ct*2, 20, (const uint8_t *)&Invader2, 24, 18, LED_BLACK, LED_GREEN);
        delay(300);  
      ledState = LOW;
    } 
}

      panel.drawLine(0, 0, 30, 25, LED_WHITE);
      delay(150);
      panel.drawLine(0, 0, 30, 25, LED_RED);
      delay(50);
      panel.drawLine(0, 0, 30, 25, LED_WHITE);
      delay(50);
      panel.drawLine(0, 0, 30, 25, LED_RED);
      delay(50);
      panel.drawLine(0, 0, 30, 25, LED_WHITE);     
      delay(100);
      panel.drawBitmap(22, 20, (const uint8_t *)&Invader2, 24, 18, LED_BLACK, LED_RED);
      delay(100);       
      panel.drawBitmap(22, 20, (const uint8_t *)&Invader2, 24, 18, LED_BLACK, LED_WHITE);
      delay(200);
   
      
for (int ct=0; ct <= 500; ct++){
      panel.drawPixel(random(0, 64), random(0, 64), random(0, 7));
      delayMicroseconds(50);
}
for (int ct=0; ct <= 1500; ct++){
      panel.drawPixel(random(0, 64), random(0, 64), LED_WHITE);
      delayMicroseconds(50);
}

      delay(50);
      FillBuffer(0xff);
      delay(250);
      FillBuffer(0x00);
      delay(150);
      FillBuffer(0xff);
      delay(150);
      FillBuffer(0x00);      
      
delay(4000);
}


