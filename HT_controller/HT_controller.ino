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

#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <Encoder.h>
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
const char C_ARROW_DOWN = 0xda;
const char C_SPACE = 0x20;

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
const int attach_sensor = 4;
const int attach_relay = 5;


// relay pins (default: 0-Heater HT, 1-Heater Fan HF, 2-Moisture Fan MF, 3-Roller RL
#define REL_0 6
#define REL_1 7
#define REL_2 8
#define REL_3 9

// auxiliary components
char FLAG[4] = {' ', ' ', ' ', ' '};
char HTU_status, DHT_status = 0x20; // used for sensors initial detection/check
float humdSetPoint = 50.0;  // humidity preset buffer
float tempSetPoint = 37.0;  // temperature preset buffer
const float C_H_LOW_LIM = 40.0;
const float C_H_HIGH_LIM = 90.0;
const float C_T_LOW_LIM = 20.0;
const float C_T_HIGH_LIM = 50.0;
int DHTS_H_i, HTUS_H_i;
const float delta_temp = 0.4;
const float delta_humd = 4.0;

struct setpointEEVault_struct {
  byte resetFlag;      // flag to control setpoint values actuality (0xA5 - values are actual, other values - setpoints are corrupted)
  float humdSetPoint;  // humidity preset between 40.0 and 90.0 (with step 0.5)
  float tempSetPoint;  // temperature preset between 30.0 and 50.0 (with step 0.1)
  byte mainSensor;     // sensor which values are used to control relays
  byte relayAsset[4];  // pin numbers for relays: 0-HT, 1-HF, 2-MF, 3-RL
};

setpointEEVault_struct setpointEEVault;
setpointEEVault_struct SV_TEST;
#define SPAdress 0


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
  pinMode(LCD_CT, OUTPUT);
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

  EEPROM.get(SPAdress, setpointEEVault);
  // DEBUG
  Serial.print(F("setting from EEPROM: "));
  Serial.print(setpointEEVault.humdSetPoint, 1);
  Serial.print(F(" - "));
  Serial.print(setpointEEVault.tempSetPoint, 1);
  Serial.print(F(" - "));
  Serial.println(setpointEEVault.mainSensor);
  // DEBUG

  if (setpointEEVault.resetFlag != 0xA5) { // EEPROM is clear - write default values
    Serial.println(F("EEPROM is clear!"));
    setpointEEVault.resetFlag = 0xA5;
    setpointEEVault.humdSetPoint = humdSetPoint;
    setpointEEVault.tempSetPoint = tempSetPoint;
    setpointEEVault.mainSensor = 0;
    EEPROM.put(SPAdress, setpointEEVault);
  } else { // setpoints are out of limits (EEPROM was corrupted)
    if ((setpointEEVault.humdSetPoint > 90.0 || setpointEEVault.humdSetPoint < 40.0) || (setpointEEVault.tempSetPoint > 50.0 || setpointEEVault.tempSetPoint < 30.0)) {
      Serial.println(F("EEPROM is corupted!"));
      setpointEEVault.resetFlag = 0xA5;
      setpointEEVault.humdSetPoint = humdSetPoint;
      setpointEEVault.tempSetPoint = tempSetPoint;
      setpointEEVault.mainSensor = 0;
      EEPROM.put(SPAdress, setpointEEVault);
    }
  }

  // show values from main sensor by default
  if (setpointEEVault.mainSensor == 0) {
    state = show_dht;
  } else {
    state = show_htu;
  }

  // delay to show sensor check status on LCD
  delay(800);
}

void show_sensor_value_LCD(const char *sensor_name, int hum_data, float temp_data, char *flag_set) {
  LCD.clear();
  LCD.home();
  LCD.print(sensor_name);
  if (hum_data < 10) {
    LCD.setCursor(7, 0);
  } else {
    if (hum_data < 100) {
      LCD.setCursor(6, 0);
    } else {
      LCD.setCursor(5, 0);
    }
  }
  LCD.print(hum_data);
  LCD.setCursor(0, 1);
  LCD.print(flag_set);
  LCD.setCursor(4, 1);
  LCD.print(temp_data, 1);
  updateLCD = false;
}

void edit_setpoint_LCD(const char *setpoint_name, float setpoint_value) {
  LCD.clear();
  LCD.home();
  LCD.print(setpoint_name);
  LCD.setCursor(4, 1);
  LCD.print(setpoint_value, 1);
  updateLCD = false;
}

void loop() {
  
  ENC.tick(); // опрос состояния энкодера. Он должен постоянно опрашиваться в цикле

  // опрос датчиков температуры и влажности
  if (millis() % 1000 == 0) { // read sensors one time in a second
    // update sensor readings
    HTUS_T = HTUS.readTemperature();
    HTUS_H = HTUS.readHumidity();
    HTUS_H_i = int(HTUS_H);
    DHTS_T = DHTS.readTemperature();
    DHTS_H = DHTS.readHumidity();
    DHTS_H_i = int(DHTS_H);
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
      Serial.print(F(" TempD:"));
      Serial.print(DHTS_T, 1);
      Serial.print(F(" HumdD:"));
      Serial.print(DHTS_H, 1);
      Serial.println();
    }
    #endif
  }

  // control relays according to chosen main sensor values
  // if Tsen < Tsp-dt/2 --> HT=on HF=on
  // if Tsen > Tsp+dt/2 --> HT=off HF=on
  // if Tsen <> {Tsp-d/2, Tsp+d/2} --> HT=off HF=off
  // if Hsen < Hsp-dh/2 --> MF=on
  // if Hsen > Hsp+dh/2 --> MF=off
  // roller should be on for shift_time 4 times per day
  
  if (setpointEEVault.mainSensor == 0) { // active sensor - DHT
    if (DHTS_T < (setpointEEVault.tempSetPoint - delta_temp/2)) { // temp is below limit
      digitalWrite(REL_0, HIGH);
      FLAG[0] = C_ARROW_DOWN;
      digitalWrite(REL_1, HIGH);
      FLAG[1] = C_ARROW_DOWN;
    } else {
      if (DHTS_T > (setpointEEVault.tempSetPoint + delta_temp/2)) { // temp is over limit
        digitalWrite(REL_0, LOW);
        FLAG[0] = C_SPACE;
        digitalWrite(REL_1, HIGH);
        FLAG[1] = C_ARROW_DOWN;
      } else { // temp is between limits
        digitalWrite(REL_0, LOW);
        FLAG[0] = C_SPACE;
        digitalWrite(REL_1, LOW);
        FLAG[1] = C_SPACE;
      }
    }
    if (DHTS_H < (setpointEEVault.humdSetPoint - delta_humd/2)) { // humidity is below limit
      digitalWrite(REL_2, HIGH);
      FLAG[2] = C_ARROW_DOWN;
    } else {
      if (DHTS_H > (setpointEEVault.humdSetPoint + delta_humd/2)) { // humidity is over limit
        digitalWrite(REL_2, LOW);
        FLAG[2] = C_SPACE;
      }
    }
  } else { // // active sensor - HTU
    if (HTUS_T < (setpointEEVault.tempSetPoint - delta_temp/2)) { // temp is below limit
      digitalWrite(REL_0, HIGH);
      FLAG[0] = C_ARROW_DOWN;
      digitalWrite(REL_1, HIGH);
      FLAG[1] = C_ARROW_DOWN;
    } else {
      if (HTUS_T > (setpointEEVault.tempSetPoint + delta_temp/2)) { // temp is over limit
        digitalWrite(REL_0, LOW);
        FLAG[0] = C_SPACE;
        digitalWrite(REL_1, HIGH);
        FLAG[1] = C_ARROW_DOWN;
      } else { // temp is between limits
        digitalWrite(REL_0, LOW);
        FLAG[0] = C_SPACE;
        digitalWrite(REL_1, LOW);
        FLAG[1] = C_SPACE;
      }
    }
    if (HTUS_H < (setpointEEVault.humdSetPoint - delta_humd/2)) { // humidity is below limit
      digitalWrite(REL_2, HIGH);
      FLAG[2] = C_ARROW_DOWN;
    } else {
      if (HTUS_H > (setpointEEVault.humdSetPoint + delta_humd/2)) { // humidity is over limit
        digitalWrite(REL_2, LOW);
        FLAG[2] = C_SPACE;
      }
    }
  }

  // main state machine - state transitions
  switch (state) {
    case show_dht:  // default state 1 - show DHT data on screen (and relay status)
      if (ENC.isRight()) {  // turn encoder to switch between two default states
        state = show_htu;
        updateLCD = true;
      }
      if (ENC.isLeft()) {  // turn encoder to switch between two default states
        state = show_htu;
        updateLCD = true;
      }
      if (ENC.isClick()) {  // press button to switch to humidity preset setup
        state = set_humd;
        updateLCD = true;
      }
      if (ENC.isHolded()) { // press and hold to switch to sensor setup mode
        state = attach_sensor;
        updateLCD = true;
      }
    break;
    case show_htu:  // default state 2 show HTU data on screen (and relay status)
      if (ENC.isRight()) {  // turn encoder to switch between two default states
        state = show_dht;
        updateLCD = true;
      }
      if (ENC.isLeft()) {  // turn encoder to switch between two default states
        state = show_dht;
        updateLCD = true;
      }
      if (ENC.isClick()) {  // press button to switch to humidity preset setup
        humdSetPoint = setpointEEVault.humdSetPoint; // update setpoint buffers before setup
        tempSetPoint = setpointEEVault.tempSetPoint; // update setpoint buffers before setup
        state = set_humd;
        updateLCD = true;
      }
      if (ENC.isHolded()) { // press and hold to switch to sensor setup mode
        state = attach_sensor;
        updateLCD = true;
      }
    break;
    case set_humd:  // set humidity preset
      if (ENC.isRight()) {
        // increment humidity preset between 40.0 and 90.0 (with step 0.5)
        if (humdSetPoint < C_H_HIGH_LIM) {
          humdSetPoint += 0.5;
        } else {
          humdSetPoint = C_H_HIGH_LIM;
        }
        updateLCD = true;
      }
      if (ENC.isLeft()) {
        // decrement humidity preset between 40.0 and 90.0 (with step 0.5)
        if (humdSetPoint > C_H_LOW_LIM) {
          humdSetPoint -= 0.5;
        } else {
          humdSetPoint = C_H_LOW_LIM;
        }
        updateLCD = true;
      }
      if (ENC.isClick()) {  // switch to temperature preset setup
        state = set_temp;
        updateLCD = true;
      }
    break;
    case set_temp:  // set temperature preset
      if (ENC.isRight()) {
        // increment temperature preset between 30.0 and 50.0 (with step 0.1)
        if (tempSetPoint < C_T_HIGH_LIM) {
          tempSetPoint += 0.1;
        } else {
          tempSetPoint = C_T_HIGH_LIM;
        }
        updateLCD = true;
      }
      if (ENC.isLeft()) {
        // decrement temperature preset between 30.0 and 50.0 (with step 0.1)
        if (tempSetPoint > C_T_LOW_LIM) {
          tempSetPoint -= 0.1;
        } else {
          tempSetPoint = C_T_LOW_LIM;
        }
        updateLCD = true;
      }
      if (ENC.isClick()) {  // save presets to EEPROM and goto show sensor values (HTU default)
        setpointEEVault.humdSetPoint = humdSetPoint;
        setpointEEVault.tempSetPoint = tempSetPoint;
        EEPROM.put(SPAdress, setpointEEVault);
        // DEBUG
        // EEPROM.get(SPAdress, SV_TEST);
        // Serial.print(F("check setting from EEPROM: "));
        // Serial.print(SV_TEST.humdSetPoint, 1);
        // Serial.print(F(" - "));
        // Serial.print(SV_TEST.tempSetPoint, 1);
        // Serial.print(F(" - "));
        // Serial.println(SV_TEST.mainSensor);
        // DEBUG
        if (setpointEEVault.mainSensor == 0) { // goto state to show default sensor values
          state = show_dht;
        } else {
          state = show_htu;
        }
        updateLCD = true;
      }
    break;
    case attach_sensor:  // set active sensor
      if (ENC.isRight()) {
        if (setpointEEVault.mainSensor == 0) {
          setpointEEVault.mainSensor = 1;
        } else {
          setpointEEVault.mainSensor = 0;
        }
        updateLCD = true;
      }
      if (ENC.isLeft()) {
        if (setpointEEVault.mainSensor == 0) {
          setpointEEVault.mainSensor = 1;
        } else {
          setpointEEVault.mainSensor = 0;
        }
        updateLCD = true;
      }
      if (ENC.isClick()) { // save chosen sensor to EEPROM and exit to default state
        EEPROM.put(SPAdress, setpointEEVault);
        // DEBUG
        // EEPROM.get(SPAdress, SV_TEST);
        // Serial.print(F("check setting from EEPROM: "));
        // Serial.print(SV_TEST.humdSetPoint, 1);
        // Serial.print(F(" - "));
        // Serial.print(SV_TEST.tempSetPoint, 1);
        // Serial.print(F(" - "));
        // Serial.println(SV_TEST.mainSensor);
        // DEBUG
        if (setpointEEVault.mainSensor == 0) {
          state = show_dht;
        } else {
          state = show_htu;
        }
        updateLCD = true;
      }
    break;
    default:
      state = show_htu;
      updateLCD = true;
    break;
  }

  // show information on LCD according to state
  if (updateLCD) {
    switch (state) {
      case show_dht:
        show_sensor_value_LCD("DHT:", DHTS_H_i, DHTS_T, FLAG);
      break;
      case show_htu:
        show_sensor_value_LCD("HTU:", HTUS_H_i, HTUS_T, FLAG);
      break;
      case set_humd:
        //edit_setpoint_LCD("Set H:", setpointEEVault.humdSetPoint);
        edit_setpoint_LCD("Set H:", humdSetPoint);
        // LCD.clear();
        // LCD.home();
        // LCD.print(F("Set H:"));
        // LCD.setCursor(4, 1);
        // LCD.print(setpointEEVault.humdSetPoint, 1);
        // updateLCD = false;
      break;
      case set_temp:
        //edit_setpoint_LCD("Set T:", setpointEEVault.tempSetPoint);
        edit_setpoint_LCD("Set T:", tempSetPoint);
        // LCD.clear();
        // LCD.home();
        // LCD.print(F("Set T:"));
        // LCD.setCursor(4, 1);
        // LCD.print(setpointEEVault.tempSetPoint, 1);
        // updateLCD = false;
      break;
      case attach_sensor:
        LCD.clear();
        LCD.home();
        LCD.print(F("Sensor:"));
        LCD.setCursor(1, 1);
        if (setpointEEVault.mainSensor == 0) {
          LCD.print("DHT-W");
        } else {
          LCD.print("HTU-R");
        }
        updateLCD = false;
      break;
    }
  }
}

