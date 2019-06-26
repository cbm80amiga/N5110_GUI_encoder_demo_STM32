/*
  GUI encoder demo
  (c)2018-19 Pawel A. Hernik
  YouTube videos:
  ATmega: https://youtu.be/GHULqZpVpz4
  STM32: https://youtu.be/YsM7WqgUP8s
*/

// Connections:
/*
 STM32:
 N5110 LCD pinout from left:
 #1 RST      - PA0
 #2 CS/CE    - PA4
 #3 DC       - PA1
 #4 MOSI/DIN - PA7
 #5 SCK/CLK  - PA5
 #6 VCC      - 3.3V
 #7 LIGHT    - 200ohm to GND or PA2
 #8 GND

 STM32 SPI1 pins:
  PA4 CS1
  PA5 SCK1
  PA6 MISO1
  PA7 MOSI1
*/

// ATmega
// N5110 LCD from left:
// #1 RST      - Pin 9
// #2 CS/CE    - Pin 10
// #3 DC       - Pin 8
// #4 MOSI/DIN - Pin 11
// #5 SCK/CLK  - Pin 13
// #6 VCC 3.3V or 5V
// #7 LIGHT
// #8 GND
// Encoder pins to 2 and 4 (uses 1st interrupt)
// Encoder push button to pin 3 (can use 2nd interrupt)
// use debouncing capacitors (100nF seems to be enough)


#include <EEPROM.h>

#if (__STM32F1__) // bluepill
#include <RTClock.h>
RTClock rtclock(RTCSEL_LSE);
tm_t curTime;
#endif

#if (__STM32F1__) // bluepill
#define N5110_RST       PA0
#define N5110_CS        PA4
#define N5110_DC        PA1
#define N5110_BACKLIGHT PA2
// use 3 debouncing capacitors (100nF seems to be enough)
#define encoderPinA     PB4
#define encoderPinB     PB5
#define encoderButton   PB6

#else // avr

#define N5110_RST       9
#define N5110_CS        10
#define N5110_DC        8
#define N5110_BACKLIGHT 6
// use 3 debouncing capacitors (100nF seems to be enough)
#define encoderPinA     2
#define encoderPinB     4
#define encoderButton   3
#endif

// define USESPI in LCD driver header for HW SPI version
#include "N5110_SPI.h"
#if USESPI==1
#include <SPI.h>
#endif
N5110_SPI lcd(N5110_RST, N5110_CS, N5110_DC); // RST,CS,DC

#include "c64enh_font.h"
#include "times_dig_16x24_font.h"
#include "term9x14_font.h"
#include "tiny3x7_font.h"
#include "small4x7_font.h"
#include "small5x7_font.h"
#include "small5x7bold_font.h"
#include "small5x6_font.h"
//#include "fonts_all.h"

int h=14,m=53,s=26,day=10,month=03,year=2019,wd=6;

#if defined (__STM32F1__) // bluepill

const char *months[] = {"???", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
const char *delim = " :";
char bld[40];

uint8_t str2month(const char * d)
{
  uint8_t i = 13;
  while((--i) && strcmp(months[i], d)) {};
  return i;
}

void setBuildTime(tm_t & mt)
{
  // Timestamp format: "Mar 3 2019 12:34:56"
  snprintf(bld,40,"%s %s\n", __DATE__, __TIME__);
  char *token = strtok(bld, delim);
  while(token) {
    int m = str2month((const char*)token);
    if(m>0) {
      mt.month = m;
      token = strtok(NULL, delim);  mt.day = atoi(token);
      token = strtok(NULL, delim);  mt.year = atoi(token) - 1970;
      token = strtok(NULL, delim);  mt.hour = atoi(token);
      token = strtok(NULL, delim);  mt.minute = atoi(token);
      token = strtok(NULL, delim);  mt.second = atoi(token);
    }
    token = strtok(NULL, delim);
  }
  snprintf(bld,40,"Build: %02d-%02d-%02d %02d:%02d:%02d\n",mt.year+1970,mt.month,mt.day,mt.hour,mt.minute,mt.second); Serial.println(bld);
  rtclock.setTime(mt);
}

#endif

// -------------------------

#if defined (__STM32F1__) // bluepill
void setup_stm32_vcc_sensor() 
{
  adc_reg_map *regs = ADC1->regs;
  regs->CR2 |= ADC_CR2_TSVREFE;
  regs->SMPR1 |=  (0b111 << 18);
  regs->SMPR1 |=  (0b111 << 21);
  adc_calibrate(ADC1);
}
#endif

// -------------------------
volatile int encoderPos = 0;

void buttonInt() {}

void readEncoderInt()
{
  (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? encoderPos++ : encoderPos--;
}

int readButton()
{
  static int lastState = HIGH;
  int val = 0, state = digitalRead(encoderButton);
  if (state == LOW && lastState == HIGH) val = 1;
  lastState = state;
  return val;
}

void initEncoder()
{
  encoderPos = 0;
  pinMode(encoderPinA,   INPUT_PULLUP);
  pinMode(encoderPinB,   INPUT_PULLUP);
  pinMode(encoderButton, INPUT_PULLUP);
  attachInterrupt(encoderPinA, readEncoderInt, CHANGE);
  attachInterrupt(encoderButton, buttonInt, CHANGE);
}

// -------------------------
// simple pixel based drawing functions
#define PIXEL_OFF 0
#define PIXEL_ON  1
#define PIXEL_XOR 2

byte scr[84 * 4]; // frame buffer
byte scrWd = 84;
byte scrHt = 4;

void clrBuf()
{
  for (int i = 0; i < scrWd * scrHt; i++) scr[i] = 0;
}

void drawPixel(int16_t x, int16_t y, uint16_t c)
{
  if ((x < 0) || (x >= scrWd) || (y < 0) || (y >= scrHt * 8)) return;
  switch (c) {
    case PIXEL_OFF: scr[x + (y / 8)*scrWd] &= ~(1 << (y & 7)); break;
    case PIXEL_ON:  scr[x + (y / 8)*scrWd] |=  (1 << (y & 7)); break;
    case PIXEL_XOR: scr[x + (y / 8)*scrWd] ^=  (1 << (y & 7)); break;
  }
}

void drawLineV(int16_t x, int16_t y0, int16_t y1, uint16_t c)
{
  if (y1 > y0) for (int y = y0; y <= y1; y++) drawPixel(x, y, c);
  else         for (int y = y1; y <= y0; y++) drawPixel(x, y, c);
}

// --------------------------------------------------------------------------

char buf[25], buf2[15];

int numT = 0;
int curT = 0;
float bufT[300];
int graphStart = 0;

char *menuTxt[] = {"MCU Temp", "VCC/Battery", "Clock", "Backlight", "Contrast", "EEPROM dump", "Graph", "Help", "Reboot"};
int numMenus = 0;
int menuLine;
int menuStart;
int numScrLines = 6;
int menuMode = -1; // -1 -> menu of options, 0..n -> option
int oldPos = 0;


void setup()
{
  Serial.begin(115200);
#if defined (__STM32F1__) // bluepill
  rtclock.breakTime(rtclock.now(), curTime);
  if(curTime.year+1970<2019) setBuildTime(curTime);  //  <2019 - invalid year
  setup_stm32_vcc_sensor();
#endif
  lcd.init();
  lcd.clrScr();
  initEncoder();
  numMenus = sizeof(menuTxt) / sizeof(char*);
  pinMode(N5110_BACKLIGHT, OUTPUT);
  analogWrite(N5110_BACKLIGHT, 0); // 0=max
}

void drawBatt(int x, int y, int wd, int perc)
{
  int w = wd * perc / 100;
  lcd.fillWin(x, y, 1 + w, 1, B01111111);
  x += w + 1;
  w = wd - w;
  if (w > 0) {
    lcd.fillWin(x, y, w, 1, B01000001);
    x += w;
  }
  lcd.fillWin(x++, y, 1, 1, B01111111);
  lcd.fillWin(x++, y, 1, 1, B00011100);
  lcd.fillWin(x++, y, 1, 1, B00011100);
}

void drawBattBig(int x, int y, int wd, int perc)
{
  int w = wd * perc / 100;
  lcd.fillWin(x, y + 0, 1, 1, B11111110);
  lcd.fillWin(x, y + 1, 1, 1, B01111111); x++;
  lcd.fillWin(x, y + 0, 1, 1, B00000010);
  lcd.fillWin(x, y + 1, 1, 1, B01000000); x++;
  lcd.fillWin(x, y + 0, w, 1, B11111010);
  lcd.fillWin(x, y + 1, w, 1, B01011111); x += w;
  w = wd - w;
  if (w > 0) {
    lcd.fillWin(x, y + 0, w, 1, B00000010);
    lcd.fillWin(x, y + 1, w, 1, B01000000); x += w;
  }
  lcd.fillWin(x, y + 0, 1, 1, B00000010);
  lcd.fillWin(x, y + 1, 1, 1, B01000000); x++;
  lcd.fillWin(x, y + 0, 1, 1, B11111110);
  lcd.fillWin(x, y + 1, 1, 1, B01111111); x++;
  lcd.fillWin(x, y + 0, 1, 1, B11100000);
  lcd.fillWin(x, y + 1, 1, 1, B00000111); x++;
  lcd.fillWin(x, y + 0, 1, 1, B11100000);
  lcd.fillWin(x, y + 1, 1, 1, B00000111);
}

// ---------------------------------------
int t = 0;
unsigned long tm;

void drawSin()
{
  if (encoderPos < 0) encoderPos = 0;
  if (encoderPos > 30) encoderPos = 30;
  float mult = encoderPos - 15;
  int x, y, yold;
  scrWd = 84;
  scrHt = 4;
  tm = millis();
  clrBuf();
  for (x = 0; x < 84; x++) {
    y = 16 + mult * (sin((x + t / 4.0) / 5.0) * cos(x / 22.0));
    if (x == 0 || y == yold)
      drawPixel(x, y, 1);
    else
      drawLineV(x, y, yold, 1);
    yold = y;
  }
  lcd.drawBuf(scr, 0, 1, scrWd, 4);
  lcd.setFont(Small5x7PLBold);
  lcd.setDigitMinWd(4);
  snprintf(buf, 99, "  Mult: %2d  ", (int)mult);
  lcd.printStr(ALIGN_CENTER, 0, buf);
  t += 4;
  while (millis() - tm < 80);
}

// ---------------------------------------
int x;
long vcc_mV = 3892;
float temp = 21.3, mint = 99, maxt = -99;
int temp1 = 21;
int temp10 = 3;

void showTemp()
{
  if(millis()-tm<500) return;
  tm = millis();
#if defined (__STM32F1__) // bluepill
  float vcc = 1.20 * 4096.0 / adc_read(ADC1, 17);
  //temp = (1.43 - (vcc / 4096.0 * adc_read(ADC1, 16))) / 0.0043 + 25.0;
  temp = (1.43 - (vcc / 4096.0 * adc_read(ADC1, 16))) / 0.0043 + 25.0 - 18;  // 18 - offset for my bluepill
#endif
  temp1 = (int)temp;
  temp10 = (temp - temp1) * 10;
  if(temp<mint) mint=temp;
  if(temp>maxt) maxt=temp;
  lcd.setFont(c64enh);
  lcd.printStr(ALIGN_CENTER, 0, "MCU Temp");
  buf[0] = 0;
  strcat(buf, " <"); dtostrf(mint, 1, 1, buf2); strcat(buf, buf2);
  strcat(buf, "' >"); dtostrf(maxt, 1, 1, buf2); strcat(buf, buf2); strcat(buf, "' ");
  lcd.printStr(ALIGN_CENTER, 5, buf);

  snprintf(buf, 10, "%d", temp1);
  lcd.setFont(times_dig_16x24);
  lcd.setDigitMinWd(17);
  x = 3;
  x = lcd.printStr(x, 1, buf);
  snprintf(buf, 10, ":%d", temp10);
  x = lcd.printStr(x, 1, buf);
  lcd.setFont(Term9x14);
  lcd.printStr(x + 1, 1, "`C");
}

void showBattery()
{
#if defined (__STM32F1__) // bluepill
  vcc_mV = 1200 * 4096 / adc_read(ADC1, 17);
#endif
  lcd.setFont(c64enh);
  lcd.printStr(ALIGN_CENTER, 0, "VCC/Battery");
  lcd.setFont(c64enh);
  lcd.setDigitMinWd(6);
  drawBattBig(8, 2, 62, constrain(map(vcc_mV, 2900, 4200, 0, 100), 0, 100));
  snprintf(buf, 8, "%d.%03d", vcc_mV / 1000, vcc_mV % 1000);
  x = lcd.printStr(20, 5, buf);
  lcd.printStr(x + 2, 5, "V");
}

void dumpEEPROM()
{
  lcd.setFont(Tiny3x7PL);
  lcd.setCharMinWd(3);
  lcd.setDigitMinWd(3);
  if (encoderPos >= (128 - 6) * 2) encoderPos = (128 - 6) * 2;
  int st = encoderPos / 2;
  for (int j = 0; j < numScrLines; j++) {
    int ii = st * 8 + j * 8;
    ii &= 0x3ff;
    snprintf(buf, 8, "%03X", ii);
    lcd.printStr(0, j, buf);
    for (int i = 0; i < 8; i++) {
      int v = EEPROM.read(ii + i) & 0xff;
      snprintf(buf, 8, "%02X", v);
      lcd.printStr(14 + i * 9, j, buf);
    }
  }
}

void showHelp()
{
  lcd.setFont(c64enh);
  lcd.printStr(ALIGN_CENTER, 0, "Help");
  lcd.setFont(Small4x7PL);
  lcd.setCR(1);
  lcd.printStr(0, 1, "Use encoder to select menu item. Press button to exit.");
  lcd.setCR(0);
}

void setBacklight()
{
  if (encoderPos > 84) encoderPos = 84;
  snprintf(buf, 6, " %d ", encoderPos * 255 / 84);
  lcd.setFont(Small5x7PL);
  lcd.setCharMinWd(5);
  lcd.setDigitMinWd(5);
  lcd.printStr(ALIGN_CENTER, 1, buf);
  lcd.printStr(ALIGN_LEFT, 1, "000");
  lcd.printStr(ALIGN_RIGHT, 1, "255");
  lcd.fillWin(0, 2, encoderPos, 1, 0xfc);
  if (encoderPos < 84) lcd.fillWin(encoderPos, 2, 84 - encoderPos, 1, 0);
  analogWrite(N5110_BACKLIGHT, 255 - encoderPos * 3);
}

void setContrast()
{
  if (encoderPos > 63) encoderPos = 63;
  snprintf(buf, 6, "%02X", encoderPos * 2);
  lcd.setFont(Small5x7PL);
  lcd.setCharMinWd(5);
  lcd.setDigitMinWd(5);
  lcd.printStr(28, 1, buf);
  lcd.printStr(ALIGN_LEFT, 1, "00");
  lcd.printStr(58, 1, "7F");
  lcd.fillWin(0, 2, encoderPos, 1, 0xfe);
  if (encoderPos < 84) lcd.fillWin(encoderPos, 2, 84 - encoderPos, 1, 0);
  lcd.setContrast(encoderPos * 2);
}

void (*avr_reset)(void) = 0;

void reboot()
{
  if (encoderPos >= 1 * 2) encoderPos = 1 * 2;
  int st = encoderPos / 2;
  lcd.setFont(c64enh);
  lcd.printStr(ALIGN_CENTER, 1, "Reboot?");
  lcd.setInvert(st ? 0 : 1);
  lcd.printStr(10, 3, " NO ");
  lcd.setInvert(st ? 1 : 0);
  lcd.printStr(43, 3, " YES ");
  lcd.setInvert(0);
  if (readButton() <= 0) return;
  menuMode = -1;
  lcd.clrScr();
  if (st > 0) { // yes
    lcd.printStr(ALIGN_CENTER, 2, "Rebooting ..."); delay(500);
    lcd.clrScr();
#if (__STM32F1__) // bluepill
    nvic_sys_reset();
#else
    avr_reset();
#endif
  }
  encoderPos = oldPos;
}

char *dowTxt[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun", "??"};
char *dowLongTxt[] = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday", "??"};

void showClock(int seconds)
{
#if defined (__STM32F1__) // bluepill
  rtclock.breakTime(rtclock.now(), curTime);
  h = curTime.hour;
  m = curTime.minute;
  s = curTime.second;
  day = curTime.day;
  month = curTime.month;
  year = curTime.year+1970;
  wd = curTime.weekday;
#endif
  lcd.setFont(Term9x14);
  if(seconds)
    snprintf(buf, 25, " %d:%02d:%02d ", h, m, s);
  else
    snprintf(buf, 25, " %d:%02d ", h, m);
  lcd.printStr(ALIGN_CENTER, 1, buf);
  lcd.setFont(c64enh);
  snprintf(buf, 25, "% 02d.%02d.%d ", day, month, year);
  lcd.printStr(ALIGN_CENTER, 4, buf);
  snprintf(buf, 25, "  %s  ", dowLongTxt[wd]);
  lcd.printStr(ALIGN_CENTER, 5, buf);
}

void setMenu(int m)
{
  menuMode = m;
  lcd.clrScr();
  oldPos = encoderPos;
  encoderPos = 0;
}

void endMenu()
{
  if (readButton() > 0) {
    menuMode = -1;
    lcd.clrScr();
    encoderPos = oldPos;
  }
}

void formatMenu(char *in, char *out, int num)
{
  int j = strlen(in);
  out[0] = ' ';
  strncpy(out + 1, in, j++);
  for (; j < num; j++) out[j] = ' ';
  out[j] = 0;
}

void drawMenuSlider()
{
  int n = (8 * numScrLines - 2 - 5 - 2) * menuLine / (numMenus - 1);
  scrWd = 3;
  scrHt = numScrLines;
  clrBuf();
  drawLineV(1, 0, numScrLines*8-1, 1);
  drawLineV(0, n+2, n+2+4, 1);
  drawLineV(2, n+2, n+2+4, 1);
  lcd.drawBuf(scr, 81, 0, scrWd, scrHt);
}

void handleMenu()
{
  //lcd.setFont(Small5x7PL);
  lcd.setFont(Small5x6PL);
  lcd.setCharMinWd(5);
  lcd.setDigitMinWd(5);
  if (encoderPos < 0) encoderPos = 0;
  if (menuMode == -1) {
    menuLine = encoderPos / 2;
    if (menuLine >= numMenus) {
      menuLine = numMenus - 1;
      encoderPos = menuLine * 2;
    }
    if (menuLine >= menuStart + numScrLines) menuStart = menuLine - numScrLines + 1;
    if (menuLine < menuStart) menuStart = menuLine;
    for (int i = 0; i < numScrLines; i++) {
      if (i + menuStart < numMenus) {
        lcd.setInvert(i + menuStart == menuLine ? 1 : 0);
        formatMenu(menuTxt[i + menuStart], buf, 13);
        lcd.printStr(ALIGN_LEFT, i, buf);
      }
    }
    drawMenuSlider();
    if (readButton()) {
      setMenu(menuLine);
      if (menuLine == 3) encoderPos = 84; // setBacklight
      if (menuLine == 4) encoderPos = 0x30 / 2; // setContrast
    }
  } else 
  if (menuMode == 0) { showTemp(); endMenu(); } else
  if (menuMode == 1) { showBattery(); endMenu(); } else
  if (menuMode == 2) { showClock(1); endMenu(); } else
  if (menuMode == 3) { setBacklight(); endMenu(); } else
  if (menuMode == 4) { setContrast(); endMenu(); } else
  if (menuMode == 5) { dumpEEPROM(); endMenu(); } else
  if (menuMode == 6) { drawSin(); endMenu(); } else
  if (menuMode == 7) { showHelp(); endMenu(); } else
  if (menuMode == 8) { reboot(); endMenu(); } else
  {  menuMode = -1; lcd.clrScr(); }
}

void loop()
{
  handleMenu();
}

