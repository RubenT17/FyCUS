 /**
  *********************************************************************************
  * @file           : CubeSat_code.ino
  * @brief          : Main program body
  * Created on      : 14.06.2022
  * Autor           : Rubén Torres Bermúdez <rubentorresbermudez@gmail.com>
  *********************************************************************************
  * @attention
  *
  *  This file provides very basic functionalities for sensoring a Cubesat for the 
  *  FYCUS university project. This project incorporates:
  *  Peripherals and devices initialization check, function check, WDT, RTC, LoRa, 
  *  GPS, IMU, acellerometer, barometer, temperature sensor, power sensor, CSV data 
  *  logger SD and data logger UART TX.
  * 
  * Last Update: 22/09/2022
  * 
  *  CubeSat_code.ino Copyright (C) 2022  Rubén Torres Bermúdez
  *********************************************************************************
  */



/* Include */
#include <SPI.h>
#include <Wire.h>
#include <Arduino_MKRGPS.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <MKRIMU.h>
#include <RTCZero.h>
#include <SD.h>
#include <LoRa.h>
#include <Adafruit_SleepyDog.h>
#include <Adafruit_INA260.h>
#include <Adafruit_ADS1X15.h>
#include <PID_v1.h>



/* Private include */
#include "MS5607B.h"
#include "GPS_mode.h"



/* Private define */
/* GPIO */
// UART: 13,14; I2C: 12,11; SPI: 10,9,8
#define ONEWIRE_PIN       7
#define CS_SD_PIN         4    // SDHD, FAT32
#define HEATER_PIN        5
#define INBUILT_LED_PIN   LED_BUILTIN //6
#define RPI_INTERRUPT_PIN 0

/* Delay time */
#define SENSOR_TIME       1000
#define LORA_TIME         5000
#define LOGGER_TIME       1100

/* WDT TIME */
#define WDT_TIME          8000

/* PID */
#define SETPOINT          2.0
#define MAX_PID           190



/* Private variables */
float GPSlatitude;
float GPSlongitude;
float GPSaltitude;
float GPSspeed;
int   GPSsatellites;
float out_press;
float out_temp;
float MSaltitude;
float x; 
float y;
float z;
float temp_bat1;
float temp_bat2;  
float temp_bat3;
double mean_temp=0.0;
float v_bat;
float i_bat;
float p_bat;
float temp_ind;
double heater_duty;         //double
double setpoint = SETPOINT; //double

/* Status variables */
bool GPS_status;
bool barometer_status;
bool IMU_status;
bool tempBat_status;
bool powerBat_status;
bool heater_status;
bool secSensors_status;
bool SD_status;
bool INA_status;

/* RTC */
const uint8_t hh=00, minut=00, sec=0;
const uint8_t dd=27, mm=7, aa=22;

/* SD variables */
String file_name = "log_";
char RTC_time [40] = {0};
char buf[1000]={0};

/* Control */
unsigned long int t1=0;
unsigned long int t2=0;
unsigned long int t3=0;

/* PID */
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

/* Interrupts flags */
bool flag_RPi=0;


/* Class Initialization */
MS5607 ms5607;
Adafruit_INA260 ina260;
OneWire Onewire(ONEWIRE_PIN);
DallasTemperature DS18B20(&Onewire);
File logFile;
RTCZero rtc;
PID PID_bat(&mean_temp, &heater_duty, &setpoint, consKp, consKi, consKd, DIRECT);



/* DS18B20 device adress */
DeviceAddress INT1      = {0x28, 0xFF, 0x64, 0x1E, 0x23, 0xA0, 0xC7, 0xBC};
DeviceAddress INT2      = {0x28, 0xFF, 0x64, 0x1E, 0x23, 0xB6, 0x51, 0x36};
DeviceAddress dir_ind1  = {0x28, 0xFF, 0x64, 0x1E, 0x23, 0xBA, 0xF5, 0xB5};



/* Private function prototypes */
bool getGPS();
bool getBarometer();
bool getIMU();
bool getTempBat();
bool getPowerBat();
bool getSecSensors();
bool setPowerHeater();
bool SD_save();
void RTC_wakeUp();
void RPi_wakeUp();
void Tx_RPi();



void setup()
{
  Serial.begin(115200);

  Serial.print(F(" ________ ___    ___ ________  ___  ___  ________    \r\n")); 
  Serial.print(F("|\\  _____\\\\  \\  /  /|\\   ____\\|\\  \\|\\  \\|\\   ____\\     \r\n"));
  Serial.print(F("\\ \\  \\__/\\ \\  \\/  / | \\  \\___|\\ \\  \\\\\\  \\ \\  \\___|_    \r\n"));
  Serial.print(F(" \\ \\   __\\\\ \\    / / \\ \\  \\    \\ \\  \\\\\\  \\ \\_____  \\   \r\n"));
  Serial.print(F("  \\ \\  \\_| \\/  /  /   \\ \\  \\____\\ \\  \\\\\\  \\|____|\\  \\  \r\n"));
  Serial.print(F("   \\ \\__\\__/  / /      \\ \\_______\\ \\_______\\____\\_\\  \\ \r\n"));
  Serial.print(F("    \\|__|\\___/ /        \\|_______|\\|_______|\\_________\\\r\n"));
  Serial.print(F("        \\|___|/                            \\|_________|\r\n\n\n"));


  Serial.println(F("INICIALIZATING PERIPHERALS AND DEVICES..."));

  pinMode(INBUILT_LED_PIN, OUTPUT);
  pinMode(CS_SD_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(RPI_INTERRUPT_PIN, INPUT_PULLDOWN);
  digitalWrite(HEATER_PIN, 0);
  attachInterrupt(digitalPinToInterrupt(RPI_INTERRUPT_PIN), Rx_RPi, RISING);


  rtc.begin();                          Serial.println(F("RTC INITIALIZATION SUCCESSFUL"));
    rtc.setHours(hh);
    rtc.setMinutes(minut);
    rtc.setSeconds(sec);
    rtc.setDay(dd);
    rtc.setMonth(mm);
    rtc.setYear(aa);

  if (!LoRa.begin(433E6))               Serial.println(F("*** ERROR LoRa ***"));
  else{                                 Serial.println(F("LoRa INITIALIZATION SUCCESSFUL"));
    LoRa.setSpreadingFactor(12);
    LoRa.setCodingRate4(8);
    LoRa.enableCrc();
    LoRa.setTxPower(20);
  }

  
  if(!ina260.begin()){                  Serial.println(F("*** ERROR INA260 ***"));
    INA_status=0;
  }
  else{                                 Serial.println(F("INA260 INITIALIZATION SUCCESSFUL"));
    ina260.setCurrentConversionTime(INA260_TIME_2_116_ms);
    ina260.setVoltageConversionTime(INA260_TIME_2_116_ms);
    ina260.setAveragingCount(INA260_COUNT_4);    
  }


  if (!GPS.begin(GPS_MODE_SHIELD))      Serial.println(F("*** ERROR GPS ***"));
  else{                                 Serial.println(F("GPS INITIALIZATION SUCCESSFUL"));
      delay(500);
      configNavigationUBX(6);
      delay(500);
    }


  if(!ms5607.begin()){                  Serial.println(F("*** ERROR MS5607B (PARALLAX) ***"));
  barometer_status=0;
  }
  else                                  Serial.println(F("MS5607B (PARALLAX) INITIALIZATION SUCCESSFUL"));


  if (!IMU.begin()){                    Serial.println(F("*** ERROR IMU ***"));
    IMU_status = 0;
  }
  else                                  Serial.println(F("IMU INITIALIZATION SUCCESSFUL "));


  if (!SD.begin(CS_SD_PIN)){            Serial.println(F("*** ERROR SD ***"));
    SD_status = 0;
  }
  else{                                 Serial.println(F("SD INITIALIZATION SUCCESSFUL"));
    String file = file_name + String("0") + String(".csv");
    logFile = SD.open(file, FILE_WRITE);
    if(!logFile)                        Serial.println(F("*** DATA LOGGER OPENING ERROR ***"));
    else{                               Serial.println(F("DATA LOGGER OPENING SUCCESSFUL"));
      logFile.write("time,latitude,longitude,GPSaltitude,speed,out_press,out_temp,");
      logFile.write("MSaltitude,x,y,z,temp_bat1,temp_bat2,v_bat,i_bat,p_bat,temp_ind\r\n");
      logFile.close();
    }
  }

  DS18B20.begin();
  DS18B20.setResolution(12);
  Serial.print(F("FOUND "));
  Serial.print(DS18B20.getDeviceCount(), DEC);
  Serial.println(F(" ONEWIRE DEVICES.\n\n\n"));

  PID_bat.SetOutputLimits(0, MAX_PID);
  PID_bat.SetMode(AUTOMATIC);

  Watchdog.enable(WDT_TIME);            Serial.println(F("WDT INITIALIZATION SUCCESSFUL"));

  uint8_t reset_cause = Watchdog.resetCause();
  Serial.print("Last reset cause: ");
  Serial.print(reset_cause, BIN);
  Serial.println("\n\n\n");
  
  LoRa.beginPacket();
  LoRa.print(reset_cause, HEX);
  LoRa.endPacket();

  Watchdog.reset();
  t1= millis();
  t2=millis();
}




void loop()
{
  Watchdog.reset();

  if ((millis() - t1) > SENSOR_TIME)
  {
    // Serial.println("INTRO SENSORES");
    digitalWrite(INBUILT_LED_PIN, HIGH);
    sprintf(RTC_time, "%02d:%02d:%02d", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    GPS_status = getGPS();
//    Serial.println("GPS OK");
    barometer_status = getBarometer();
    IMU_status = getIMU();
    // Serial.println("IMU OK");
    tempBat_status = getTempBat();
    // Serial.println("DS OK");
    powerBat_status = getPowerBat();
    // Serial.println("INA OK");
    heater_status = setPowerHeater();  // Hacer comprobación de cuantas veces se le manda un 1 --> Fallo Temp   // O ver con millis() si sube la temp o no
    // Serial.println("Heater OK");
    secSensors_status = getSecSensors();
    // Serial.println("SecSens OK");
    debug_variables();
    debug_states();
    // Serial.println("Debug OK");
    digitalWrite(INBUILT_LED_PIN, LOW);
    // Serial.println("PIN OK");
    t1=millis();
  }

  if (millis() - t2 > LOGGER_TIME)
  {
    logger_RPi();
    // Serial.println("INTRO SD");
    SD_status = SD_save();
    // Serial.println("SD OK");
    t2=millis();
  }
  
  if (millis() - t3 > LORA_TIME)
  {
    // Serial.println("INTRO LORA");
    LoRa_Tx();
    t3=millis();
  }

  if(flag_RPi)
  {
    Rx_RPi();
    flag_RPi=0;
  }
  
}


















/**
* @brief Receive I2C data from GPS
* @returns true if success
*/
bool getGPS()
{
  unsigned long int t = millis();
  
  while (!GPS.available())
  {
    if((millis()-t) > 500) return 0;
  }
    GPSlatitude     = GPS.latitude();
    GPSlongitude    = GPS.longitude();
    GPSaltitude     = GPS.altitude();
    GPSspeed        = GPS.speed();  // km/h
    GPSsatellites   = GPS.satellites();
    return 1;
}

/**
* @brief Receive I2C data from MS5607 sensor (barometer)
* @returns true if success
*/
bool getBarometer()
{
  if(!barometer_status)
  {
    if(!ms5607.begin()) return 0;
  }
  if (ms5607.readDigitalValue())  
  {
    out_temp    = ms5607.getTemperature();  //ºC
    out_press   = ms5607.getPressure();     //mbar
    MSaltitude  = ms5607.getAltitude();     //m
    return 1;  
  }
  else return 0;
}

/**
* @brief Receive I2C data from IMU
* @returns true if success
*/
bool getIMU()
{
  if(!IMU_status)
  {
    if(!IMU.begin()) return 0;
  }
  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(x, y, z);
    return 1; 
  }
  else return 0;
}

/**
* @brief Receiva OneWire data from DS18B20 sensors (battery temp sensors)
* @returns true if success
*/
bool getTempBat()
{
  DS18B20.requestTemperatures();

  temp_bat1 = DS18B20.getTempC(INT1);
  delayMicroseconds(15);
  temp_bat2 = DS18B20.getTempC(INT2);
  delayMicroseconds(15);
  temp_ind = DS18B20.getTempC(dir_ind1);
  delayMicroseconds(15); 
  mean_temp = (temp_bat1+temp_bat2)/2;

  if (temp_bat1 == DEVICE_DISCONNECTED_C || temp_bat2 == DEVICE_DISCONNECTED_C) return 0;
  else return 1;
}


/**
* @brief Receive I2C data from INA sensor (battery)
* @returns true if success
*/
bool getPowerBat()
{
  if(!INA_status)
  {
    if(!ina260.begin()) return 0;
  }
  v_bat = ina260.readBusVoltage()/1000;   //V
  i_bat = ina260.readCurrent();           //mA
  p_bat = ina260.readPower();             //mW 
  if (v_bat<13.7 || i_bat>10000.0)  return 0;
  else return 1;
}

/**
* @brief Measure secondary sensors
* @returns true if success
*/
bool getSecSensors()
{
  
  return 1;
}


/**
* @brief Save data in SD card (SPI)
* @returns true if agresive PID is set
*/
bool setPowerHeater()
{
  double gap = abs(setpoint-mean_temp);
  bool ret;
  if (gap < 10)
  {
    PID_bat.SetTunings(consKp, consKi, consKd);
    ret=0;
  }
  else
  {
     PID_bat.SetTunings(aggKp, aggKi, aggKd);
     ret=1;
  }
  PID_bat.Compute();
  analogWrite(HEATER_PIN, heater_duty);
  return ret;
}



/**
* @brief Transmit functions states with LoRa packet
* @returns none
*/
void LoRa_Tx()
{  
  uint8_t func_status = GPS_status<<7 | barometer_status<<6 | IMU_status<<5 | tempBat_status<<4 |
                        powerBat_status<<3 | heater_status<<2 | secSensors_status<<1 | SD_status;
  LoRa.beginPacket();
  LoRa.print(func_status, HEX);
  LoRa.endPacket();
}


/**
* @brief Save data in SD card
* @returns true if success
*/
bool SD_save()
{
  static unsigned int count = 0;
  static unsigned int num = 0;
  String file;
  
  count++;
  if(count>200)
  {
    num++;
    count=1;
    file = file_name + String(num) + String(".csv");
    logFile = SD.open(file, FILE_WRITE);
    if(logFile)
    {
      logFile.write("time,latitude,longitude,GPSaltitude,speed,out_press,out_temp,");
      logFile.write("MSaltitude,x,y,z,temp_bat1,temp_bat2,v_bat,i_bat,p_bat,temp_ind\r\n");
      logFile.close();
    }
    else  return 0;
  }
  else 
  file = file_name + String(num) + String(".csv");
  
  logFile = SD.open(file, FILE_WRITE);
  if(logFile)
  {
    int len = sprintf(buf, "%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,", 
    RTC_time,GPSlatitude, GPSlongitude, GPSaltitude, GPSspeed, out_press, out_temp, MSaltitude);
    logFile.write(buf, len); 
       
    len = sprintf(buf, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", 
    x,  y, z, temp_bat1, temp_bat2, v_bat, i_bat, p_bat, temp_ind);
    logFile.write(buf, len);
    
    logFile.close();
  }
  else  return 0;

  return 1;
}



/**
* @brief Interrupt of RPi request
* @returns None
*/
void Rx_RPi()
{
  flag_RPi = 1;
}

/**
* @brief Transmission data to RPi
* @returns None
*/
void Tx_RPi()
{
  char buff[50]={0};
  sprintf(buff, "%.2f,%.2f,%.2f,%.2f\r\n", GPSlatitude, GPSlongitude, GPSaltitude, GPSspeed);
  Serial.print(buff);
}

/**
* @brief Transmission data logger to RPi
* @returns None
*/
void logger_RPi()
{
    int len = sprintf(buf, "%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", 
    RTC_time,GPSlatitude, GPSlongitude, GPSaltitude, GPSspeed, out_press, out_temp, MSaltitude, x,  y, z, temp_bat1, temp_bat2, v_bat, i_bat, p_bat, temp_ind);
    Serial.write(buf, len);
}

/**
* @brief Debug funcions states 
* @returns None
*/
void debug_states()
{
  Serial.println("Estado de las funciones:");
  uint8_t func_status = GPS_status<<7 | barometer_status<<6 | IMU_status<<5 | tempBat_status<<4 |
                    powerBat_status<<3 | heater_status<<2 | secSensors_status<<1 | SD_status;
  Serial.println(func_status, BIN);
  Serial.println("\n");

}


/**
* @brief Debug variables
* @returns None
*/
void debug_variables()
{
  Serial.println("Valores de las variables:");
  Serial.println(GPSlatitude);
  Serial.println(GPSlongitude);
  Serial.println(GPSaltitude);
  Serial.println(GPSspeed);
  Serial.println(GPSsatellites);
  Serial.println(out_press);
  Serial.println(out_temp);
  Serial.println(MSaltitude);
  Serial.println(x); 
  Serial.println(y);
  Serial.println(z);
  Serial.println(temp_bat1);
  Serial.println(temp_bat2); 
  Serial.println(mean_temp); 
  Serial.println(temp_ind);
  Serial.println(heater_duty);
  Serial.println(v_bat);
  Serial.println(i_bat);
  Serial.println(p_bat);
  Serial.println();
  Serial.println(millis());
  Serial.println("\n");
}
