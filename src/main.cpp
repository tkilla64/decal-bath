/*
 * Project: Heat controller for a decal bath.
 * Author: Tommy Killander (tommy.killander@gmail.com)
 *
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DS18B20.h>
#include <U8x8lib.h>
#include <PID_v1.h>

// pin definitions
#define MENU_BUTTON     2
#define UP_BUTTON       3
#define DOWN_BUTTON     4
#define ONE_WIRE_BUS    5
#define HEATER          6
#define ACT_LED1        7
#define ACT_LED2        8
#define CURR_MEAS       A0

#define DEBOUNCE_TIME   25

// Dynamic power consumption is calculated from voltage and current.
// Uncomment HAS_CURR_MEAS if electronics has a current measurement circuit,
// otherwise current will be approximated from PWM value.
#define HAS_CURR_MEAS

// Shunt resistors are R13 and R14
#define SHUNT_RESISTOR  0.05f

// Gain factor in the measurement op amp (R19 / R17)
#define OPAMP_GAIN      10.0f

// Use a multimeter to measure the ADC analog reference on AtMega528P pin 20
// Should be around ~1.100 V
#define AREF            1.084f

// Use a multimeter to measure the voltage across the heater during full heater
// mode (the frontpanel LED is Red)
#define HEATER_VOLT     14.90f

// Use multimeter to measure the resistance of the Heatbed. Make sure that it
// is disconnected from the PCB during he measurement
#define HEATER_RESISTOR 8.1f

#define PWM_LOW         0.0f
#define PWM_HIGH        255.0f

//
// OLED display configuration (vendor dependant)
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE); // pchbutik.se #1443
//U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C u8x8(U8X8_PIN_NONE);

OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire);

// Uncomment for more debug information
//#define DEBUG

// Limits for parameters
#define LOW_TEMP        30.0f
#define HIGH_TEMP       50.0f
#define LOW_SPLTIME     1
#define HIGH_SPLTIME    30
#define LOW_KP          1.0f
#define HIGH_KP         1000.0f
#define LOW_KI          0.0f
#define HIGH_KI         10.0f
#define LOW_KD          0.0f
#define HIGH_KD         1000.0f

// Keymask for buttons
#define UP              1
#define DOWN            2
#define MENU            4

// Menu levels
#define MAIN            0
#define SETSAMPLE       1
#define SETKP           2
#define SETKI           3
#define SETKD           4
#define SETCF           5
#define STORE           6

struct nonVolatileData {
  double tempSetPoint;   // target temperature
  double kp;             // P coefficient gain
  double ki;             // I coefficient 1/s
  double kd;             // D coefficient s
  char tempUnitCF;       // 'C' or 'F'
  int sampleTimeS;       // sample time in s
  int magic;             // magic number when data is valid
};

nonVolatileData EEdata;

int pkp, pki, pkd;      // printout coefficients
int tempRead, tempSetp;
double currentMeas, outputPower;
char string[64] = "";
int buttons, lastButtons;
int menuLevel = MAIN;
double tempReading;      // measured temperature from sensor
double heaterControl;    // PID result to drive PWM output

PID myPID(&tempReading, &heaterControl, &EEdata.tempSetPoint,
  EEdata.kp, EEdata.ki, EEdata.kd, DIRECT);

//
// Local functions
float readTemperature(void)
{
  sensor.requestTemperatures();
  while (!sensor.isConversionComplete())
  {
    delay(2);
  }

  return sensor.getTempC();
}

int readButtons()
{
  int keyBitMask = 0;
  if (digitalRead(UP_BUTTON) == 0)
  {
    keyBitMask += UP;
  }
  if (digitalRead(DOWN_BUTTON) == 0)
  {
    keyBitMask += DOWN;
  }
  if (digitalRead(MENU_BUTTON) == 0)
  {
    keyBitMask += MENU;
  }
  return keyBitMask;
}

float convertCtoF(float celsius)
{
  return ((celsius * 1.8f) + 32.0f);
}


void setup()
{
  pinMode(MENU_BUTTON, INPUT);
  pinMode(UP_BUTTON, INPUT);
  pinMode(DOWN_BUTTON, INPUT);
  pinMode(ONE_WIRE_BUS, INPUT);
  pinMode(HEATER, OUTPUT);
  pinMode(ACT_LED1, OUTPUT);
  pinMode(ACT_LED2, OUTPUT);

  // for PWM frequency of 7812.50 Hz
  //TCCR0B = (TCCR0B & B11111000) | B00000010;

  analogReference(INTERNAL);
  sensor.begin();
  u8x8.begin();

  EEPROM.get(0, EEdata);
  if (EEdata.magic != 12345)
  {
    EEdata.tempSetPoint = 40.0f;
    EEdata.kp = 15.0f;
    EEdata.ki = 0.05f;
    EEdata.kd = 0.0f;
    EEdata.sampleTimeS = 1;
    EEdata.tempUnitCF = 'C';
    EEdata.magic = 12345;
  }

  // Init PID regulator
  myPID.SetTunings(EEdata.kp, EEdata.ki, EEdata.kd);
  myPID.SetOutputLimits(PWM_LOW, PWM_HIGH);
  myPID.SetSampleTime(EEdata.sampleTimeS * 1000);
  tempReading = readTemperature();
  myPID.SetMode(AUTOMATIC);

#ifdef DEBUG
  // logging on serial port
  Serial.begin(9600);

  pkp = (int)EEdata.kp;
  pki = (int)(EEdata.ki * 100);
  pkd = (int)EEdata.kd;
  sprintf(string, "setpoint;temp;pwm;P:%d I*100:%d D:%d S:%d", pkp, pki, pkd,
    EEdata.sampleTimeS);
  Serial.println(string);
#endif
}

void loop()
{
  // Read temperature and call PID algorithm
  tempReading = readTemperature();
  myPID.Compute();
  // Control heater
  analogWrite(HEATER, (int)heaterControl);

#ifdef DEBUG
  if ((millis() % 1000) < 200)
  {
    // logging on serial port
    tempRead = (int)(tempReading * 10);
    sprintf(string, "%d;%02d,%1d;%d", (int)EEdata.tempSetPoint,
      tempRead / 10, tempRead % 10, (int)heaterControl);
    Serial.println(string);
  }
#endif

  // Output temp reading and setpoint on display
  if (EEdata.tempUnitCF == 'C')
  {
    tempRead = (int)(tempReading);
    tempSetp = (int)EEdata.tempSetPoint;
  }
  else
  {
    tempRead = (int)convertCtoF(tempReading);
    tempSetp = (int)convertCtoF(EEdata.tempSetPoint);
  }
  sprintf(string, "%d%c%c ", tempRead, 176, EEdata.tempUnitCF);
  u8x8.setFont(u8x8_font_courB18_2x3_f);
  u8x8.drawString(0, 3, string);
  sprintf(string, "Setpoint: %3d%c%c ", tempSetp, 176, EEdata.tempUnitCF);
  u8x8.setFont(u8x8_font_7x14_1x2_f);
  u8x8.drawString(0, 0, string);

  // Read buttons and debounce
  buttons = readButtons();
  delay(DEBOUNCE_TIME);
  if (buttons != readButtons())
  {
    buttons = 0;
  }

  // Menu system
  switch (menuLevel)
  {
    case MAIN:
      if (buttons == UP)
      {
        EEdata.tempSetPoint += 1.0f;
      }
      if (buttons == DOWN)
      {
        EEdata.tempSetPoint -= 1.0f;
      }
      if (buttons == MENU)
      {
        menuLevel = SETSAMPLE;
      }
      EEdata.tempSetPoint = constrain(EEdata.tempSetPoint, LOW_TEMP, HIGH_TEMP);
      EEPROM.put(0, EEdata);


#ifdef HAS_CURR_MEAS
      currentMeas = ((AREF / 1023) * analogRead(CURR_MEAS))
                    / (OPAMP_GAIN * SHUNT_RESISTOR);
#else
      // if we have no current measurement circuit we approximate the current
      // from the PWM value
      currentMeas = ((HEATER_VOLT / HEATER_RESISTOR)
                    / (float)(PWM_HIGH - PWM_LOW)) * heaterControl;
#endif
      outputPower = currentMeas * HEATER_VOLT;
      sprintf(string, "Heater pwr:%2dW ", (int)outputPower);

      // Control status LEDs
      if (heaterControl == 255)
      {
        digitalWrite(ACT_LED1, 0);
        digitalWrite(ACT_LED2, 1);
      }
      else if (heaterControl == 0)
      {
        digitalWrite(ACT_LED1, 1);
        digitalWrite(ACT_LED2, 0);
      }
      else
      {
        digitalWrite(ACT_LED1, 0);
        digitalWrite(ACT_LED2, 0);
      }
      break;

    case SETSAMPLE:
      if (buttons == UP)
      {
        EEdata.sampleTimeS += 1.0f;
      }
      if (buttons == DOWN)
      {
        EEdata.sampleTimeS -= 1.0f;
      }
      if (buttons == MENU && lastButtons == 0)
      {
        menuLevel = SETKP;
      }
      EEdata.sampleTimeS = constrain(EEdata.sampleTimeS, LOW_SPLTIME, HIGH_SPLTIME);
      sprintf(string, "Sample time:%2ds", (int)EEdata.sampleTimeS);
      break;

    case SETKP:
      if (buttons == UP)
      {
        EEdata.kp += 1.0f;
      }
      if (buttons == DOWN)
      {
        EEdata.kp -= 1.0f;
      }
      if (buttons == MENU && lastButtons == 0)
      {
        menuLevel = SETKI;
      }
      EEdata.kp = constrain(EEdata.kp, LOW_KP, HIGH_KP);
      sprintf(string, "Set P (Kp):%4d", (int)EEdata.kp);
      break;

    case SETKI:
      if (buttons == UP)
      {
        EEdata.ki += 0.01f;
      }
      if (buttons == DOWN)
      {
        EEdata.ki -= 0.01f;
      }
      if (buttons == MENU && lastButtons == 0)
      {
        menuLevel = SETKD;
      }
      EEdata.ki = constrain(EEdata.ki, LOW_KI, HIGH_KI);
      sprintf(string, "Set I (Ki):%4d", (int)(EEdata.ki*100));
      break;

    case SETKD:
      if (buttons == UP)
      {
        EEdata.kd += 1.0f;
      }
      if (buttons == DOWN)
      {
        EEdata.kd -= 1.0f;
      }
      if (buttons == MENU && lastButtons == 0)
      {
        menuLevel = SETCF;
      }
      EEdata.kd = constrain(EEdata.kd, LOW_KD, HIGH_KD);
      sprintf(string, "Set D (Kd):%4d", (int)EEdata.kd);
      break;

    case SETCF:
      if (buttons == UP)
      {
        EEdata.tempUnitCF = 'C';
      }
      if (buttons == DOWN)
      {
        EEdata.tempUnitCF = 'F';
      }
      if (buttons == MENU && lastButtons == 0)
      {
        menuLevel = STORE;
      }
      sprintf(string, " +%cC       -%cF ", 176, 176);
      break;

    case STORE:
      if (buttons == UP)
      {
        EEPROM.put(0, EEdata);
        myPID.SetTunings(EEdata.kp, EEdata.ki, EEdata.kd);
        myPID.SetSampleTime(EEdata.sampleTimeS * 1000);
        menuLevel = MAIN;
      }
      if (buttons == DOWN)
      {
        EEPROM.get(0, EEdata);
        menuLevel = MAIN;
      }
      sprintf(string, "+Store   -Abort");
      break;

    default:
      break;
  }
  u8x8.setFont(u8x8_font_7x14_1x2_f);
  u8x8.drawString(0, 6, string);
  lastButtons = buttons;
}
