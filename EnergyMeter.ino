#include "ATM90E36.h"
#include <SPI.h>
#include <U8g2lib.h>
#include <U8x8lib.h>


#define m90e36a
#define serialCalibration
#define DEBUG

//*********SPI M90E36A***********
#define CS 7
#define SCK PA5
#define MISO PA6
#define MOSI PA7
#define DMA PB10
//*********RS-485 DIRECTION******
#define DIR PB13
//*********I2C OLED**************
#define SCL PB6
#define SDA PB7
//*********ENCODER PINS**********
#define A PB3
#define B PB4
#define OK PB5
//*********WATER MEATER**********
#define WATER PB1
#define WaterConst 1  //multyply by this constant
//*********GAS MEATER************
#define GAS PB0
#define GasConst 1  //multyply by this constant

#define EncoderLimit 9
#define displayTime 15000000000
#define refreshTime 15000000000
#define refDispTime 1000000
//*******************GLOBAL VARIABLES******************************
volatile boolean loaded = false, displayPower = false, A_set = true, B_set = true;
volatile byte EncoderPos = 1, ia = 1;
volatile double gas = 0, water = 0, ImpEnergy = 0, ExpEnergy = 0;
#ifdef m90e36a
ATM90E36 eic(CS);
#endif
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ PB8, /* data=*/ PB9, /* reset=*/ U8X8_PIN_NONE);   // Všechny displeje bez RES-pinu
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL, SDA);// [page buffer, size = 128 bytes]
HardwareTimer displayTimer(1);
HardwareTimer refreshTimer(2);
HardwareTimer refDispTimer(3);



void print();
void display(byte i);
void valueFormat(double v, char n[4],  char *c);
void doWater();
void doGas();
void doEncoderA();
void doEncoderB();
void doEncoderOK();
void displayOff();
void refDisp();
void refresh();
void calibrationPrint();

//int h=0, m=0, s=0;
//int ia=1;

void setup(void) {
  pinMode(DMA, OUTPUT);
  digitalWrite(DMA, LOW); 
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  
  u8g2.begin();
  #ifdef m90e36a
  eic.begin();
  #endif
  Serial.begin(115200);
  Serial2.begin(115200);
  displayTimer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  refreshTimer.setMode(TIMER_CH2, TIMER_OUTPUT_COMPARE);
  refDispTimer.setMode(TIMER_CH4, TIMER_OUTPUT_COMPARE);
  displayTimer.pause();
  refreshTimer.pause();
  refDispTimer.pause();
  displayTimer.setPeriod(displayTime);
  refreshTimer.setPeriod(refreshTime);
  refDispTimer.setPeriod(refDispTime);
  displayTimer.attachCompare1Interrupt(displayOff);
  refreshTimer.attachCompare2Interrupt(refresh);
  refDispTimer.attachCompare4Interrupt(refDisp);
  displayTimer.refresh();
  refreshTimer.refresh();
  refDispTimer.refresh();
  displayTimer.setCount(0);
  refreshTimer.setCount(0);
  refDispTimer.setCount(0);
  displayTimer.resume();
  #ifndef serialCalibration
  refreshTimer.resume();
  #endif
  refDispTimer.resume();
  displayPower = true;
  display(EncoderPos);


  pinMode(DMA, OUTPUT);
  //pinMode(PC13, OUTPUT);
  digitalWrite(DMA, LOW); //
  pinMode(WATER, INPUT_PULLUP);
  attachInterrupt(WATER, doWater, FALLING);
  pinMode(GAS, INPUT_PULLUP);
  attachInterrupt(GAS, doGas, FALLING);
  pinMode(A, INPUT_PULLUP);
  //A_set = digitalRead(A);
  attachInterrupt(A, updateEncoder, CHANGE);
  pinMode(B, INPUT_PULLUP);
  attachInterrupt(B, updateEncoder, CHANGE);
  pinMode(OK, INPUT_PULLUP);
  attachInterrupt(OK, doEncoderOK, FALLING);
}
void loop(void) {
  if (Serial2.available() > 0) {
    // read the incoming byte:
    char inChar = (char)Serial2.read();

    // say what you got:
    //Serial.print("Serial 1 received: ");
    Serial.print(inChar);
}
}


void display(byte i) {
  u8g2.clearBuffer();
  double Ad, Bd, Cd, Dd;
  #ifndef m90e36a
    Ad = 123.1234;
    Bd= 10000.36;
    Cd=0.32164;
    Dd=0.32164;
  #endif
  char a[12], b[12], c[12], d[12];
  //String a;
  switch (i)
  {
    case 1:
      #ifdef m90e36a
        Ad = eic.GetLineVoltageA();
        Bd = eic.GetLineVoltageB();
        Cd = eic.GetLineVoltageC();
      #endif
      
      valueFormat(Ad, "V",  &a[0]);
      valueFormat(Bd, "V",  &b[0]);
      valueFormat(Cd, "V",  &c[0]);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 14, "U");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(16, 14, "L1");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 14, a);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 30, "U");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(16, 30, "L2");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 30, b);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 46, "U");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(16, 46, "L3");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 46, c);
      break;

    case 2:
      #ifdef m90e36a
        Ad = eic.GetLineCurrentA();
        Bd = eic.GetLineCurrentB();
        Cd = eic.GetLineCurrentC();
        Dd = eic.GetLineCurrentN();
      #endif
      valueFormat(Ad, "A",  &a[0]);
      valueFormat(Bd, "A",  &b[0]);
      valueFormat(Cd, "A",  &c[0]);
      valueFormat(Dd, "A",  &d[0]);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 14, "I");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(9, 14, "L1");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 14, a);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 30, "I");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(9, 30, "L2");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 30, b);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 46, "I");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(9, 46, "L3");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 46, c);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 62, "I");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(9, 62, "N");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 62, d);
      break;

    case 3:
      #ifdef m90e36a
        Ad = eic.GetActivePowerA();
        Bd = eic.GetActivePowerB();
        Cd = eic.GetActivePowerC();
        Dd = eic.GetTotalActivePower();
      #endif
      valueFormat(Ad, "W",  &a[0]);
      valueFormat(Bd, "W",  &b[0]);
      valueFormat(Cd, "W",  &c[0]);
      valueFormat(Dd, "W",  &d[0]);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 14, "P");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(13, 14, "L1");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 14, a);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 30, "P");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(13, 30, "L2");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 30, b);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 46, "P");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(13, 46, "L3");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 46, c);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 62, "P");
      u8g2.setFont(u8g2_font_courR10_tr);
      //u8g2.drawStr(13,62,"TOT");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 62, d);
      break;

    case 4:
      #ifdef m90e36a
        Ad = eic.GetReactivePowerA();
        Bd = eic.GetReactivePowerB();
        Cd = eic.GetReactivePowerC();
        Dd = eic.GetTotalReactivePower();
      #endif
      valueFormat(Ad, "var",  &a[0]);
      valueFormat(Bd, "var",  &b[0]);
      valueFormat(Cd, "var",  &c[0]);
      valueFormat(Dd, "var",  &d[0]);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 14, "Q");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(17, 14, "L1");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 14, a);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 30, "Q");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(17, 30, "L2");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 30, b);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 46, "Q");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(17, 46, "L3");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 46, c);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 62, "Q");
      u8g2.setFont(u8g2_font_courR10_tr);
      //u8g2.drawStr(13,62,"TOT");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 62, d);
      break;

    case 5:
      #ifdef m90e36a
        Ad = eic.GetApparentPowerA();
        Bd = eic.GetApparentPowerB();
        Cd = eic.GetApparentPowerC();
        Dd = eic.GetTotalApparentPower();
      #endif
      valueFormat(Ad, "VA",  &a[0]);
      valueFormat(Bd, "VA",  &b[0]);
      valueFormat(Cd, "VA",  &c[0]);
      valueFormat(Dd, "VA",  &d[0]);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 14, "S");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(13, 14, "L1");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 14, a);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 30, "S");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(13, 30, "L2");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 30, b);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 46, "S");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(13, 46, "L3");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 46, c);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 62, "S");
      u8g2.setFont(u8g2_font_courR10_tr);
      //u8g2.drawStr(13,62,"TOT");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(34, 62, d);
      break;

    case 6:
      #ifdef m90e36a
        Ad = eic.GetPowerFactorA();
        Bd = eic.GetPowerFactorB();
        Cd = eic.GetPowerFactorC();
        Dd = eic.GetTotalPowerFactor();
      #endif
      valueFormat(Ad, "",  &a[0]);
      valueFormat(Bd, "",  &b[0]);
      valueFormat(Cd, "",  &c[0]);
      valueFormat(Dd, "",  &d[0]);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 14, "PF");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(28, 14, "L1");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(47, 14, a);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 30, "PF");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(28, 30, "L2");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(47, 30, b);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 46, "PF");
      u8g2.setFont(u8g2_font_courR10_tr);
      u8g2.drawStr(28, 46, "L3");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(47, 46, c);

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 62, "PF");
      u8g2.setFont(u8g2_font_courR10_tr);
      //u8g2.drawStr(13,62,"TOT");
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(47, 62, d);
      break;

    case 7:
      #ifdef m90e36a
        ImpEnergy += eic.GetImportEnergy();
        Ad = ImpEnergy;
        ExpEnergy += eic.GetExportEnergy();
        Bd = ExpEnergy;
      #endif
      

      u8g2.setFont(u8g2_font_ncenB12_tr);
      u8g2.drawStr(0, 14, "Imp energy:");

      sprintf(a, " %.2f kWh", (float)Ad);
      u8g2.drawStr(0, 30, a);

      u8g2.drawStr(0, 46, "Exp energy:");

      sprintf(b, " %.2f kWh", (float)Bd);
      u8g2.drawStr(0 , 62, b);
      break;

    case 8:
      #ifdef m90e36a
        Ad = eic.GetFrequency();
        Bd = eic.GetTemperature();
      #endif
      

      u8g2.setFont(u8g2_font_ncenB12_tr);
      u8g2.drawStr(0, 14, "Frequecny:");

      sprintf(a, " %.3f Hz", (float)Ad);
      u8g2.drawStr(0, 30, a);

      u8g2.drawStr(0, 46, "Temperature:");

      sprintf(b, " %.0f °C", (float)Bd);
      u8g2.drawStr(0 , 62, b);
      break;

    case 9:
      u8g2.setFont(u8g2_font_ncenB12_tr);
      u8g2.drawStr(0, 14, "Water:");

      sprintf(a, " %.2f m^3", (float)water);
      u8g2.drawStr(0, 30, a);

      u8g2.drawStr(0, 46, "Gas:");

      sprintf(b, " %.2f m^3", (float)gas);
      u8g2.drawStr(0 , 62, b);
      break;


  }
  u8g2.sendBuffer();
}


void valueFormat(double v, char n[3],  char *c) { //value, sufix, formatted char array
  double vl = v;
  
  if (v >= 10000 || v <= -10000) {
    if (n[1] == 0) sprintf(c, ": %.0f %s", (float)v, n);
    else if (n[2] == 0)  sprintf(c, ": %.0f%s", (float)v, n);
    else  sprintf(c, ": %.0f%s", (float)v, n);
  }
  else if (v >= 1000 || v <= -1000) {
    if (n[1] == 0) sprintf(c, ": %.1f %s", (float)v, n);
    else if (n[2] == 0)  sprintf(c, ": %.0f %s", (float)v, n);
    else  sprintf(c, ": %.0f%s", (float)v, n);
  }
  else if (v >= 100 || v <= -100) {
    if (n[1] == 0) sprintf(c, ": %.2f %s", (float)v, n);
    else if (n[2] == 0)  sprintf(c, ": %.1f %s", (float)v, n);
    else  sprintf(c, ": %.0f %s", (float)v, n);
  }
  else if (v >= 10 || v <= -10) {
    if (n[1] == 0) sprintf(c, "%.3f %s", (float)v, n);
    else if (n[2] == 0)  sprintf(c, ": %.2f %s", (float)v, n);
    else  sprintf(c, ": %.1f %s", (float)v, n);
  }
  else {
    if (n[1] == 0) sprintf(c, ": %.4f %s", (float)v, n);
    else if (n[2] == 0) sprintf(c, ": %.3f %s", (float)v, n);
    else sprintf(c, ": %.2f %s", (float)v, n);
  }
  //return();
}

void doWater() {
  delay(100);
  if (!digitalRead(WATER)){
    water += WaterConst;
    #ifdef DEBUG
    Serial.println(water);
    #endif
  }
  
}
void doGas() {
  delay(10);
  if (!digitalRead(GAS)){
    gas += GasConst;
    #ifdef DEBUG
    Serial.println(water);
    #endif
  }
}

void refDisp() {
  #ifdef DEBUG
  Serial.println("Display refreshed");
  #endif
  display(EncoderPos);
}

void updateEncoder() {
  boolean a = digitalRead(A);
  boolean b = digitalRead(B);
  if (A_set && !B_set && a && b && displayPower) {
    EncoderPos++;
    if (EncoderPos == 0) EncoderPos = EncoderLimit;
    if (EncoderPos > EncoderLimit) EncoderPos = 1;
    #ifdef DEBUG
    Serial.println(EncoderPos, DEC);
    #endif
    displayTimer.pause();
  displayTimer.setCount(0);
  displayTimer.resume();
    display(EncoderPos);
  }
  if (!A_set && B_set && a && b && displayPower) {
    EncoderPos--;
    if (EncoderPos == 0) EncoderPos = EncoderLimit;
    if (EncoderPos > EncoderLimit) EncoderPos = 1;
    #ifdef DEBUG
    Serial.println(EncoderPos, DEC);
    #endif
    displayTimer.pause();
  displayTimer.setCount(0);
  displayTimer.resume();
    display(EncoderPos);
  }

  A_set = a;
  B_set = b;
}


void doEncoderOK() {
  #ifdef DEBUG
  Serial.println("OK");
  #endif
  //refresh();
  if (!displayPower) {
    #ifdef DEBUG
    Serial.println("Display turn ON");
    #endif
    displayPower = true;
    u8g2.setPowerSave(0);
    //displayTimer.resume();
  }
  
  display(EncoderPos);
  displayTimer.pause();
  refDispTimer.pause();
  displayTimer.setCount(0);
  refDispTimer.setCount(0);
  displayTimer.resume();
  refDispTimer.resume();
  #ifdef serialCalibration
    calibrationPrint();
  #endif
}

void displayOff() {
  #ifdef DEBUG
  Serial.println("Display off");
  #endif
  u8g2.setPowerSave(1);
  displayPower = false;
  refDispTimer.pause();
  displayTimer.pause();
}

void refresh() {
  #ifdef DEBUG
  Serial.println("Refresh");
  //Serial2.println("&U_L1=10.1");
  #endif
  #ifndef serialCalibration
  #ifdef m90e36a
    Serial2.println("&U_L1=" + String(eic.GetLineVoltageA()));
    Serial2.println("&U_L2=" + String(eic.GetLineVoltageB()));
    Serial2.println("&U_L3=" + String(eic.GetLineVoltageC()));
    delay(500);
    Serial2.println("&I_L1=" + String(eic.GetLineCurrentA()));
    Serial2.println("&I_L2=" + String(eic.GetLineCurrentB()));
    Serial2.println("&I_L3=" + String(eic.GetLineCurrentC()));
    Serial2.println("&I_N=" + String(eic.GetLineCurrentN()));
    delay(500);
    Serial2.println("&P_L1=" + String(eic.GetActivePowerA()));
    Serial2.println("&P_L2=" + String(eic.GetActivePowerB()));
    Serial2.println("&P_L3=" + String(eic.GetActivePowerC()));
    Serial2.println("&P=" + String(eic.GetTotalActivePower()));
    delay(500);
    Serial2.println("&Q_L1=" + String(eic.GetReactivePowerA()));
    Serial2.println("&Q_L2=" + String(eic.GetReactivePowerB()));
    Serial2.println("&Q_L3=" + String(eic.GetReactivePowerC()));
    Serial2.println("&Q=" + String(eic.GetTotalReactivePower()));
    delay(500);
    Serial2.println("&S_L1=" + String(eic.GetApparentPowerA()));
    Serial2.println("&S_L2=" + String(eic.GetApparentPowerB()));
    Serial2.println("&S_L3=" + String(eic.GetApparentPowerC()));
    Serial2.println("&S=" + String(eic.GetTotalApparentPower()));
    delay(500);
    Serial2.println("&Frequency=" + String(eic.GetFrequency()));
    Serial2.println("&Temperature=" + String(eic.GetTemperature()));
    delay(500);
    Serial2.println("&PE_L1=" + String(eic.GetPowerFactorA()));
    Serial2.println("&PE_L2=" + String(eic.GetPowerFactorB()));
    Serial2.println("&PE_L3=" + String(eic.GetPowerFactorC()));
    Serial2.println("&PE=" + String(eic.GetTotalPowerFactor()));
    delay(500);
    Serial2.println("&Phase_L1=" + String(eic.GetPhaseA()));
    Serial2.println("&Phase_L2=" + String(eic.GetPhaseB()));
    Serial2.println("&Phase_L3=" + String(eic.GetPhaseC()));
    delay(500);
    ImpEnergy += eic.GetImportEnergy();
    ExpEnergy += eic.GetExportEnergy();
    Serial2.println("&E_imported=" + String(ImpEnergy));
    Serial2.println("&E_exported=" + String(ExpEnergy));
    delay(500);
    Serial1.println("&Water=" + String(water));
    Serial1.println("&Gas=" + String(gas));
    /*Serial1.println("&=" + String(eic.GetSysStatus0()));
    Serial1.println("&=" + String(eic.GetSysStatus1()));
    Serial1.println("&=" + String(eic.GetMeterStatus0()));
    Serial1.println("&=" + String(eic.GetMeterStatus1()));*/
  #endif
  #endif
  
}

void calibrationPrint() {
  refDispTimer.pause();
  displayTimer.pause();
  #ifdef m90e36a
    Serial.println("------------------------------------------------------------------------------------------");
    Serial.print("Systemstatus0 = " +String(eic.GetSysStatus0()));
    Serial.println("Calibration Error=" + String(eic.calibrationError()));
    Serial.println("");
    
    Serial.println("U_L1=" + String(eic.GetLineVoltageA()));
    Serial.println("U_L2=" + String(eic.GetLineVoltageB()));
    Serial.println("U_L3=" + String(eic.GetLineVoltageC()));
    Serial.println("");
    delay(400);
    Serial.println("I_L1=" + String(eic.GetLineCurrentA()));
    Serial.println("I_L2=" + String(eic.GetLineCurrentB()));
    Serial.println("I_L3=" + String(eic.GetLineCurrentC()));
    Serial.println("I_N=" + String(eic.GetLineCurrentN()));
    Serial.println("");
    delay(400);
    
    Serial.println("P_L1=" + String(eic.GetActivePowerA()));
    Serial.println("P_L2=" + String(eic.GetActivePowerB()));
    Serial.println("P_L3=" + String(eic.GetActivePowerC()));
    Serial.println("P=" + String(eic.GetTotalActivePower()));
    Serial.println("");
    delay(400);
    Serial.println("Q_L1=" + String(eic.GetReactivePowerA()));
    Serial.println("Q_L2=" + String(eic.GetReactivePowerB()));
    Serial.println("Q_L3=" + String(eic.GetReactivePowerC()));
    Serial.println("Q=" + String(eic.GetTotalReactivePower()));
    Serial.println("");
    delay(400);
    Serial.println("S_L1=" + String(eic.GetApparentPowerA()));
    Serial.println("S_L2=" + String(eic.GetApparentPowerB()));
    Serial.println("S_L3=" + String(eic.GetApparentPowerC()));
    Serial.println("S=" + String(eic.GetTotalApparentPower()));
    Serial.println("");
    delay(400);
    Serial.println("PE_L1=" + String(eic.GetPowerFactorA()));
    Serial.println("PE_L2=" + String(eic.GetPowerFactorB()));
    Serial.println("PE_L3=" + String(eic.GetPowerFactorC()));
    Serial.println("PE=" + String(eic.GetTotalPowerFactor()));
    Serial.println("");
    delay(400);
    Serial.println("Phase_L1=" + String(eic.GetPhaseA()));
    Serial.println("Phase_L2=" + String(eic.GetPhaseB()));
    Serial.println("Phase_L3=" + String(eic.GetPhaseC()));
    Serial.println("");
    delay(400);
    Serial.println("Calibration Error=" + String(eic.calibrationError()));
    Serial.println("");
    
  #endif
  refDispTimer.resume();
  displayTimer.resume();
}
