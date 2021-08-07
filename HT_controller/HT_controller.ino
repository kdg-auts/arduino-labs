/*
Temperature and Humidity Controller
Reads T and H data from sensors HTU21D and AM2302 (DHT22), compare them with setup values
and drive rinse and heating relays (fans) whichever is nessesary. Settings are adjusted by
encoder with button and displayed on LCD screen (8x2).

** Hardware description

1) main controller - Arduino Nano v.3

2) LCD 14 pin 8 columns 2 rows - 4-pin connection is used
   pin settings:
   1 = GND
   2 = VCC (5V)
   3 = VO -- digital pin 10 (brightness ajust PWM)
   4 = RS -- digital pin 12
   5 = RW -- GND
   6 = EN -- digital pin 11
   11 = D4 -- digital pin 5
   12 = D5 -- digital pin 4
   13 = D6 -- digital pin 3
   14 = D7 -- digital pin 2

3) encoder
   GND = GND
   +   = VCC (5V)
   SW  = A2 / digital pin 16
   DT  = A1 / digital pin 15
   CLK = A0 / digital pin 14

4) sensor HTU21D
   3.3v = 3V3
   GND = GND
   SDA = A4 / digital pin 18
   SCL = A5 / digital pin 19

5) sensor AM2302 (DHT22)
   GND = GND
   VCC = VCC (5V)
   DAT = A3 / digital pin 17

6) relay outputs
   R1-IN = digital pin 6
   R2-IN = digital pin 7
   R3-IN = digital pin 8
   R4-IN = digital pin 9

*/

#include <LiquidCrystal.h>
#include <GyverEncoder.h>
#include <Wire.h>
#include "SparkFunHTU21D.h"
#include "DHT.h"

// переключатель режима отладки
#define HT_DEBUG_MODE 1

// инициализация экрана LCD
#define LCD_RS 12
#define LCD_EN 11
#define LCD_CT 10
#define LCD_D4 5
#define LCD_D5 4
#define LCD_D6 3
#define LCD_D7 2

LiquidCrystal LCD(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
bool updateLCD = false;
int contrast = 0;

// Encoder init
#define ENC_CK 14
#define ENC_DT 15
#define ENC_SW 16

Encoder ENC(ENC_CK, ENC_DT, ENC_SW); 

// Create an instance of the DHT22 sensor object
#define DHT_PIN 17
#define DHT_TYPE DHT22   // DHT22 (AM2302), AM2321
DHT DHTS(DHT_PIN, DHT_TYPE);
float DHTS_H, DHTS_T = 0;

// Create an instance of the HTU21D sensor object
HTU21D HTUS;
float HTUS_H, HTUS_T = 0;

// main FSM definition
int state = 0;
const int show_dht = 0;
const int show_htu = 1;
const int set_humd = 2;
const int set_temp = 3;

// relay pins
#define REL_0 6
#define REL_1 7
#define REL_2 8
#define REL_3 9

// auxiliary components
char FLAG[4] = {' ', ' ', ' ', ' '};
char HTU_status, DHT_status = 0x20;
float testSetPoint = 0.0;
int DHTS_H_i, HTUS_H_i;


void setup() {
  Serial.begin(9600); // init serial port for debug
  
  ENC.setType(TYPE1); // тип энкодера TYPE1 одношаговый, TYPE2 двухшаговый

  // relay pins configure
  pinMode(REL_0, OUTPUT);
  pinMode(REL_1, OUTPUT);
  pinMode(REL_2, OUTPUT);
  pinMode(REL_3, OUTPUT);
  
  LCD.begin(8, 2); // set up the LCD's number of columns and rows
  LCD.setCursor(1, 0); // set the cursor to column 1, line 0
  //pinMode(LCD_CT, OUTPUT);
  analogWrite(LCD_CT, contrast); // set max contrast
  HTUS.begin(); // turn on and init HTU21D sensor
  DHTS.begin(); // turn on and init AM2302 sensor
  
  // check if sensors are connected and work properly
  LCD.print("CHECK:");
  HTUS_T = HTUS.readTemperature();
  DHTS_T = DHTS.readTemperature();
  if (HTUS_T > 990) HTU_status = 'x'; else HTU_status = 'v';
  if (isnan(DHTS_T)) DHT_status = 'x'; else DHT_status = 'v';
  LCD.setCursor(0, 1);
  LCD.print("HTU"); LCD.print(HTU_status);
  LCD.print("DHT"); LCD.print(DHT_status);
  delay(800);
}

void loop() {
  
  ENC.tick(); // опрос состояния энкодера. Он должен постоянно опрашиваться в цикле

  // опрос датчиков температуры и влажности
  if (millis() % 1000 == 0) { // read sensors one time in a second
    // update sensor readings
    HTUS_H = HTUS.readHumidity();
    HTUS_T = HTUS.readTemperature();
    DHTS_H = DHTS.readHumidity();
    DHTS_T = DHTS.readTemperature();
    updateLCD = true; // trigger LCD update to refresh displayed values
    
    // DEBUG output to Serial
    #ifdef HT_DEBUG_MODE
    if (HTUS_T == 998 || HTUS_H == 998) {
      Serial.println(F("Failed to read from HTU sensor! Check wiring!"));
    } else {
      Serial.print(F(" TempU:"));
      Serial.print(HTUS_T, 1);
      Serial.print(F(" HumdU:"));
      Serial.print(HTUS_H, 1);
    }
    if (isnan(DHTS_H) || isnan(DHTS_T)) {
      Serial.println(F("Failed to read from DHT sensor! Check wiring!"));
    } else {
      //Serial.print(F("DHT: "));
      //Serial.print(millis()/1000);
      Serial.print(F(" TempD:"));
      Serial.print(DHTS_T, 1);
      Serial.print(F(" HumdD:"));
      Serial.print(DHTS_H, 1);
      Serial.println();
    }
    #endif
  }
  
  if (ENC.isRight()) {
    switch (state) {
      case show_dht:
        state = show_htu;
        updateLCD = true;
      break;
      case show_htu:
        state = show_dht;
        updateLCD = true;
      break;
      case set_temp:
        
        updateLCD = true;
      break;
      case set_humd:
        
        updateLCD = true;
      break;
      default:
        state = show_htu;
        updateLCD = true;
      break;
    }
    /*if (testSetPoint < 40) {
      testSetPoint += 0.1;
    } else {
      testSetPoint = 40;
    }*/
  }
  
  if (ENC.isLeft()) {
    switch (state) {
      case show_dht:
        state = show_htu;
        updateLCD = true;
      break;
      case show_htu:
        state = show_dht;
        updateLCD = true;
      break;
      case set_temp:
        
        updateLCD = true;
      break;
      case set_humd:
        
        updateLCD = true;
      break;
      default:
        state = show_htu;
        updateLCD = true;
      break;
    }
    /*if (testSetPoint > -40) {
      testSetPoint -= 0.1;
    } else {
      testSetPoint = -40;
    }*/
  }

  if (ENC.isClick()) {
    //testSetPoint = 0.0;
    if (FLAG[0] == ' ') {
      FLAG[0] = 0xda; 
      digitalWrite(REL_0, HIGH);
    } else {
      FLAG[0] = ' '; 
      digitalWrite(REL_0, LOW);
    }
    if (FLAG[1] == ' ') FLAG[1] = 0xda; else FLAG[1] = ' ';
    if (FLAG[2] == ' ') FLAG[2] = 0xda; else FLAG[2] = ' ';
    if (FLAG[3] == ' ') FLAG[3] = 0xda; else FLAG[3] = ' '; 
    /*switch (state) {
      case show_dht:
        state = set_temp;
        updateLCD = true;
      break;
      case show_htu:
        state = set_temp;
        updateLCD = true;
      break;
      case set_temp:
        state = set_temp;
        updateLCD = true;
      break;
      case set_humd:
        
        updateLCD = true;
      break;
      default:
        state = show_htu;
        updateLCD = true;
      break;
    }*/
    updateLCD = true;
  }
  
  if (updateLCD) {
    switch (state) {
      case show_dht:
        LCD.clear();
        LCD.home();
        LCD.print(F("DHT:"));
        DHTS_H_i = int(DHTS_H);
        if (DHTS_H_i < 10) {
          LCD.setCursor(7, 0);
        } else {
          if (DHTS_H_i < 100) {
            LCD.setCursor(6, 0);
          } else {
            LCD.setCursor(5, 0);
          }
        }
        LCD.print(DHTS_H_i);
        LCD.setCursor(0, 1);
        LCD.print(FLAG);
        LCD.setCursor(4, 1);
        LCD.print(DHTS_T, 1);
        //LCD.leftToRight();
        updateLCD = false;
      break;
      case show_htu:
        LCD.clear();
        LCD.home();
        LCD.print(F("HTU:"));
        HTUS_H_i = int(HTUS_H);
        if (HTUS_H_i < 10) {
          LCD.setCursor(7, 0);
        } else {
          if (HTUS_H_i < 100) {
            LCD.setCursor(6, 0);
          } else {
            LCD.setCursor(5, 0);
          }
        }
        LCD.print(HTUS_H_i);
        LCD.setCursor(0, 1);
        LCD.print(FLAG);
        LCD.setCursor(4, 1);
        LCD.print(HTUS_T, 1);
        //LCD.leftToRight();
        updateLCD = false;
      break;
    }
    /*LCD.clear();
    LCD.setCursor(0, 1);
    LCD.print(testSetPoint, 1);
    updateLCD = false;*/
  }
}
