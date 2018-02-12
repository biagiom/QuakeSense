/*
    QuakeSense_Node_LowPower_TinyGPS

    QuakeSense is an IoT project that aims to "sense" and monitor
    earthquakes through the measures provided by the LSM6DSL accelerometer
    and a Lora network used to send alarms and environmental data.
    
    This is a development version of the QuakeSense_Node sketch
    in fact it use the STM32LowPower library in order to put the
    Node in deep sleep low-power mode (STM32 stop mode) in order
    to save battery energy.
    The node wakes up from low-poer mode using interrupt generated by
    the LSM6DSL accelerometer or using RTC in order to send
    environmental data to the LoRa gateway.
    Instead of using the Adafruit GPS Library this sketch includes the
    TinyGPS++ Arduino library to parse the data received from the GPS module
    
    The main elements of this project are:
    1) At least one sensor node consisting of:
       - a 3-axis accelerometer used to measure seismic waves result
         in the strong ground motion.
       - a LoRa radio module used to transmit an alarm and data.
         coming from the accelermoter to a single channel LoRa gateway
       - A GPS module used to get altitude, longitude and latitude of
         the position of the seismic event.
       - a development board used to control the accelerometer,
         the LoRa module and the GPS module.
       - a LiPo battery used to power the development board.

       In this implementation, the sensor node has been made with:
       > LSM6DSL - 3D accelerometer and 3D gyroscope
       > STM32 Nucleo F401RE as the development board
       > Dragino LoRa/GPS Shield including
         - a 137 MHz to 1020 MHz Low Power Long Range LoRa RF Transceiver
         - a L80 GPS module based on MTK MT3339

    2) A LoRa gateway used to receive environmental data and messages
       from the sensor nodes and to send them to a cloud platform.

       In this implementation, the LoRa gateway has been made with:
       > B-L475E-IOT01A2 STM32L4 Discovery kit IoT node featuring
         - Wi-Fi® module Inventek ISM43362-M3G-L44 (802.11.4 b/g/n compliant)
         - Sub-GHz (868 Mhz) low-power RF module SPSGRF-868
         - Bluetooth® V4.1 module (SPBTLE-RF)
         - Capacitive humidity and temperature sensor (HTS221)
         - 3-axis magnetometer (LIS3MDL)
         - 3D accelerometer and 3D gyroscope (LSM6DSL)
         - 260-1260 hPa absolute digital output barometer (LPS22HB)
       > Dragino LoRa Shield which includes:
         - the SX1276 868 Mhz Low Power Long Range LoRa RF Transceiver

    3) A cloud platform in order to allow the user to visualize
       the eartquake status, alarm messages and environmental data.

       In this implementation, Adafruit IO has been choosen as the
       cloud platform and data coming from the LoRa gateway is sent
       to Adafruit IO using the MQTT protocol.

    Copyright (C) Biagio Montaruli <biagio.hkr@gmail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

*/

// Include sensor's library
#include <HTS221Sensor.h>
#include <LPS22HBSensor.h>
#include <LSM6DSLSensor.h>
#include <STM32LowPower.h>
// Include TinyGPS++ library to get location, date and time
// from the Quectel L80 GPS module
#include <TinyGPS++.h>
// include SPI library and LoRa library
#include <SPI.h>
#include <LoRa.h>

// Change these values according to your implementation
#define ENABLE_ENV_SENSORS 1
#define PRINT_ENV_DATA 1
#define SEND_ENV_DATA_AFTER_EARTHQUAKE 0
#define SEND_ENV_DATA 1
#define DEBUG_LORA_PACKET 1
#define READ_GYRO 0
#define PRINT_GYRO_DATA 0
#define PRINT_ACC_DATA  0
#define PRINT_GPS_DATA 1
#define GPS_FIX_TIME 1
#define SERIAL_DEBUG  0
#define PRINT_EARTHQUAKE_VALUES 1

#if (PRINT_ENV_DATA == 1)  || (DEBUG_LORA_PACKET == 1) || \
    (PRINT_GYRO_DATA == 1) || (PRINT_ACC_DATA == 1)    || \
    (PRINT_GPS_DATA == 1)  || (SERIAL_DEBUG == 1)      || \
    (PRINT_EARTHQUAKE_VALUES == 1)
#define ENABLE_SERIAL 1
#else
#define ENABLE_SERIAL 0
#endif

// Two available methods to get samples from the acceleometer:
// 1st METHOD (1):
// - start reading the samples from the accelerometer only if 
//   at least one of the values related to the 3 components 
//   of the motion (x, y, z) is above the treshold
// - end when all the values of the 3 components 
//   of the motion (x, y, z) are below the treshold
//
// 2nd METHOD (2):
// - start reading the values of the accelerometer only if 
//   at least one of the values related to the 3 components 
//   of the motion (x, y, z) is above the treshold
// - end when MAX_SAMPLES samples have been read
// NOTE: each sample is composed by 3 value and each value
// represents one of the 3 components (x, y, z) of acceleration
#define METHOD_TO_READ_SAMPLES 1

#define PGHAX_THRESHOLD 80
#define PGHAY_THRESHOLD 80
#define PGVA_THRESHOLD 1150

bool earthquakeDetected = false;
volatile bool motionDetected = false;

#define DEV_I2C Wire

// Use Serial2 port (USART2 on PA1, PA0) to print data in the Serial Monitor
#define SerialPort Serial

// The TinyGPS++ object
TinyGPSPlus gps;
static const uint16_t GPSBaud = 9600;
// For GPS data use USART6 port (RX -> PA_12, TX -> PA_11)
HardwareSerial GPSSerial(PA_12, PA_11);

#define GPS_READ_TIME 1000
bool isGPSDataValid = false;
typedef enum {
  GPS_LOCATION = 0,
  GPS_DATE = 1,
  GPS_TIME = 2,
  GPS_NUM_VALUES = 3
} GPS_VALUES;

bool GPSDataStatus[GPS_NUM_VALUES];

const int csPin = 10;         // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

String data;    // outgoing message send through the LoRa module

byte msgCount = 0;                // count of sent messages
byte localAddress = 0xAA;         // address of this device (transmitter - LoRa Node)
byte destinationAddress = 0xBB;   // address of the receiver (LoRa gateway)

bool loraInit = false;

// Objects for environmental sensors
#if ENABLE_ENV_SENSORS == 1
HTS221Sensor *HTS221_HumTemp;
LPS22HBSensor *LPS22HB_Press;

bool getEnvData = true;

// time interval to update the environmental data:
// temperature, humidity and pressure: 15 minutes
#define ENVDATA_UPDATE_TIME_MIN 15

#define ENVDATA_UPDATE_TIME_MILLIS (ENVDATA_UPDATE_TIME_MIN * 60000)
#endif

LSM6DSLSensor *LSM6DSL_AccGyro;

// Output data rate of LSM6DSL Accelerometer
// Available values: 13 Hz, 26 Hz, 52 Hz, 104 Hz, 208 Hz, 416 Hz,
//                   833 Hz, 1660 Hz, 3330 Hz, 6660 Hz
#define LSM6DSL_ACC_ODR LSM6DSL_ACC_GYRO_ODR_XL_104Hz
// Full scale range of LSM&DSL Accelerometer
// Available values: 2g, 4g, 8g, 16g
#define LSM6DSL_ACC_FS  LSM6DSL_ACC_GYRO_FS_XL_2g

// Maximum number of samples read by the accelerometer
#define MAX_SAMPLES 256

typedef enum {
  LSM6DSL_X_AXIS = 0,
  LSM6DSL_Y_AXIS = 1,
  LSM6DSL_Z_AXIS = 2,
  LSM6DSL_NUM_AXIS = 3
} LSM6DSL_ACC_AXIS;

#if READ_GYRO == 1
typedef enum {
  LSM6DSL_GYRO_ROLL = 0,
  LSM6DSL_GYRO_PITCH = 1,
  LSM6DSL_GYRO_YAW = 2,
  LSM6DSL_GYRO_NUM_DATA = 3
} LSM6DSL_GYRO_DATA;
#endif

volatile uint32_t counter = 0;

const int wakeupPin = PA0;

void setup() {
  
#if ENABLE_SERIAL == 1
  // Initialize Serial communication at 115200 bps.
  SerialPort.begin(115200);
  // wait for the Serial port to open
  while (!SerialPort) ;
#endif

  delay(1000);

  // Initialize I2C bus.
  DEV_I2C.begin();

  LowPower.begin();

  // Enable interrupts for LSM6DSL accelerometer sensor
  // LowPower.attachInterrupt(D5, event_cb, RISING);
  LowPower.attachInterruptWakeup(wakeupPin, motionEventCB, RISING);

  // Enable and initialize sensors.
#if ENABLE_ENV_SENSORS == 1
  // Create a new object that represents the HTS221 humidity and
  // temperature sensors and enable them
  HTS221_HumTemp = new HTS221Sensor(&DEV_I2C);
  HTS221_HumTemp->Enable();
  // Create a new object that represents the LPS22HB pressure
  // sensor and enable it
  LPS22HB_Press = new LPS22HBSensor(&DEV_I2C);
  LPS22HB_Press->Enable();
#endif

  // Create a new object that represents the LSM6DSL accelemometer
  // gyroscope and enable them
  LSM6DSL_AccGyro = new LSM6DSLSensor(&DEV_I2C);
  LSM6DSL_AccGyro->Enable_X();
  
  // Set LSM6DSL Accelerometer full scale
  LSM6DSL_AccGyro->Set_X_FS(LSM6DSL_ACC_FS);
  // Set LSM6DSL Accelerometer output data rate
  LSM6DSL_AccGyro->Set_X_ODR(LSM6DSL_ACC_ODR);
  
#if READ_GYRO == 1
  LSM6DSL_AccGyro->Enable_G();
#endif

  // Enable wake up detection
  LSM6DSL_AccGyro->Enable_Wake_Up_Detection();

  // set CS, reset, IRQ pins for LoRa module
  LoRa.setPins(csPin, resetPin, irqPin);

  // Set LoRa frequency band to 868 MHz
  if (!LoRa.begin(868E6)) {
    SerialPort.println("LoRa init failed. Check your connections.");
  }
  SerialPort.println("LoRa radio successfully initialized.");
  loraInit = true;
  // Set LoRa Mode 3
  // BW = 125 kHz; CR = 4/5; SF = 10
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  // 9600 NMEA is the default baud rate for MTK GPS's
  GPSSerial.begin(GPSBaud);
#if SERIAL_DEBUG == 1
  SerialPort.print("Using TinyGPS++ library v");
  SerialPort.println(TinyGPSPlus::libraryVersion());
#endif
  delay(1000);

  updateGPSData();
}

void loop() {

#if ENABLE_ENV_SENSORS == 1
  if (getEnvData == true) {
    // Read humidity and temperature from HTS221 sensor.
    float hts221_humidity, hts221_temperature;
    HTS221_HumTemp->GetHumidity(&hts221_humidity);
    HTS221_HumTemp->GetTemperature(&hts221_temperature);

    // Read pressure and temperature from LPS22HB sensor.
    float lps22hb_pressure;
    LPS22HB_Press->GetPressure(&lps22hb_pressure);

    // Output data to the Serial monitor and if SEND_ENV_DATA is
    // equals to 1 send humidity, temperature and pressure values
    // to the LoRa gateway
  #if PRINT_ENV_DATA == 1
    SerialPort.print("Humidity[%]: ");
    SerialPort.println(hts221_humidity, 2);
    SerialPort.print("Temperature[°C]: ");
    SerialPort.println(hts221_temperature, 2);
    SerialPort.print("Presure[mbar]: ");
    SerialPort.println(lps22hb_pressure, 2);
  #endif

  #if SEND_ENV_DATA == 1
    // Build LoRa message
    String msg = "#TEMP=" + String(hts221_temperature, 2) +
                 "#HUM=" + String(hts221_humidity, 2) +
                 "#PRESS=" + String(lps22hb_pressure, 2);
  #if DEBUG_LORA_PACKET == 1
    Serial.println("Lora packet sent:");
    Serial.println(msg);
  #endif    // DEBUG_LORA_PACKET
  sendLoraPacket(msg);
  #endif    // SEND_ENV_DATA
  }
  #endif   // ENABLE_ENV_SENSORS

#if SERIAL_DEBUG == 1
  SerialPort.println("Starting Deep Sleep low-power mode (STM32 Standby mode)...");
  delay(1000);
#endif
  
  LowPower.deepSleep(ENVDATA_UPDATE_TIME_MILLIS);

#if SEND_ENV_DATA_AFTER_EARTHQUAKE == 1
  if (motionDetected == true) {
    if (earthquakeDetected == true) {
      getEnvData = true;
      earthquakeDetected = false;
      delay(2000);
    }
    else {
      getEnvData = false;
    }
    motionDetected = false;
  }
  else {
  #if SERIAL_DEBUG == 1
    SerialPort.println("Waking up from Deep Sleep mode...");
  #endif
    getEnvData = true;
  }
#else
  if (motionDetected == true) {
    getEnvData = false;
    earthquakeDetected = false;
    motionDetected = false;
  }
  else {
  #if SERIAL_DEBUG == 1
    SerialPort.println("Waking up from Deep Sleep mode...");
  #endif
    getEnvData = true;
  }
#endif
}

void motionEventCB() {
  LSM6DSL_Event_Status_t status;
  LSM6DSL_AccGyro->Get_Event_Status(&status);

#if SERIAL_DEBUG == 1
  SerialPort.println("Waking up from Deep Sleep mode...");
#endif

  counter++;

  if (status.WakeUpStatus) {
  #if SERIAL_DEBUG == 1
    SerialPort.println("WakeUP event -> Motion Detected!");
  #endif
    motionDetected = true;
    earthquakeDetection();
  }
}

void earthquakeDetection() {
  uint32_t quake_start_time = 0;
  uint32_t quake_end_time = 0;
  uint32_t duration = 0;

  // Read accelerometer and gyroscope.
  int32_t lsm6dsl_acc[MAX_SAMPLES][LSM6DSL_NUM_AXIS];
#if READ_GYRO == 1
  int32_t lsm6dsl_gyro[MAX_SAMPLES][LSM6DSL_GYRO_NUM_DATA];
#endif

  uint16_t n;

  for (n = 0; n < MAX_SAMPLES; n++) {
    LSM6DSL_AccGyro->Get_X_Axes(lsm6dsl_acc[n]);

    if ( (abs(lsm6dsl_acc[n][LSM6DSL_X_AXIS]) >= PGHAX_THRESHOLD) || \
         (abs(lsm6dsl_acc[n][LSM6DSL_Y_AXIS]) >= PGHAY_THRESHOLD) || \
         (abs(lsm6dsl_acc[n][LSM6DSL_Z_AXIS]) >= PGVA_THRESHOLD) ) {

      if (n == 0) {
        quake_start_time = millis();
      }

    #if READ_GYRO == 1
      LSM6DSL_AccGyro->Get_G_Axes(lsm6dsl_gyro[n]);
    #endif

    }
    else {

    #if METHOD_TO_READ_SAMPLES == 1
      break;
    #elif METHOD_TO_READ_SAMPLES == 2
      if (n == 0) {
        break;
      }
    #endif
    }
  }
  quake_end_time = millis();
  
  if (n > 0) {
    duration = quake_end_time - quake_start_time;

    earthquakeDetected = true;

  #if PRINT_ACC_DATA == 1
    for (uint16_t i = 0; i < n; i++) {
      SerialPort.println("Accelerometer[mg]:");
      SerialPort.print("x-axis: ");
      SerialPort.println(lsm6dsl_acc[i][LSM6DSL_X_AXIS]);
      SerialPort.print("y-axis: ");
      SerialPort.println(lsm6dsl_acc[i][LSM6DSL_Y_AXIS]);
      SerialPort.print("z-axis: ");
      SerialPort.println(lsm6dsl_acc[i][LSM6DSL_Z_AXIS]);
    }
  #endif

  #if (READ_GYRO == 1) && (PRINT_GYRO_DATA == 1)
    for (uint16_t i = 0; i < n; i++) {
      SerialPort.println("Gyroscope[dps]:");
      SerialPort.print("roll: ");
      SerialPort.println((float) lsm6dsl_gyro[i][LSM6DSL_GYRO_ROLL] / 1000, 2);
      SerialPort.print("pitch: ");
      SerialPort.println((float) lsm6dsl_gyro[i][LSM6DSL_GYRO_PITCH] / 1000, 2);
      SerialPort.print("yaw: ");
      SerialPort.println((float) lsm6dsl_gyro[i][LSM6DSL_GYRO_YAW] / 1000, 2);
    }
  #endif

    // calculate peak ground horizontal acceleration (PGHA)
    // for x-axis and y-axis
    uint32_t pgha_x = abs(lsm6dsl_acc[0][LSM6DSL_X_AXIS]);
    uint32_t pgha_y = abs(lsm6dsl_acc[0][LSM6DSL_Y_AXIS]);

    // peak ground vertical acceleration (z-axis): PGVA
    uint32_t pgva = abs(lsm6dsl_acc[0][LSM6DSL_Z_AXIS]);

    for (uint16_t i = 1; i < n; i++) {
      if (pgha_x < abs(lsm6dsl_acc[i][LSM6DSL_X_AXIS])) {
        pgha_x = abs(lsm6dsl_acc[i][LSM6DSL_X_AXIS]);
      }
      if (pgha_y < abs(lsm6dsl_acc[i][LSM6DSL_Y_AXIS])) {
        pgha_y = abs(lsm6dsl_acc[i][LSM6DSL_Y_AXIS]);
      }
      if (pgva < abs(lsm6dsl_acc[i][LSM6DSL_Z_AXIS])) {
        pgva = abs(lsm6dsl_acc[i][LSM6DSL_Z_AXIS]);
      }
    }

  #if PRINT_EARTHQUAKE_VALUES == 1
    SerialPort.print("Earthquake duration: ");
    SerialPort.print(duration);
    SerialPort.println(" msecs");
    
    SerialPort.print("PGHA (x-axis): ");
    SerialPort.println(pgha_x);
    SerialPort.print("PGHA (y-axis): ");
    SerialPort.println(pgha_y);
    SerialPort.print("PGVA (z-axis): ");
    SerialPort.println(pgva);

    SerialPort.print("Got ");
    SerialPort.print(n);
    SerialPort.println(" samples.");
  #endif

    updateGPSData();

    if (loraInit) {
      String msg = "#ALERT!";
      msg += "#PGHAX=";
      msg += String(pgha_x);
      msg += "#PGHAY=";
      msg += String(pgha_y);
      msg += "#PGVA=";
      msg += String(pgva);
      msg += "#DUR=";
      msg += String(duration);

      if (isGPSDataValid) {
        msg += "#LAT=";
        msg += String(gps.location.lat());
        msg += "#LON=";
        msg += String(gps.location.lng());
        msg += "#ALT=";
        msg += String(gps.altitude.meters(), 2);
        msg += "#DATE=";
        if (gps.date.day() < 10)
          msg += "0";
        msg += (String(gps.date.day()) + "/");
        if (gps.date.month() < 10)
          msg += "0";
        msg += (String(gps.date.month()) + "/20");
        if (gps.date.year() < 10)
          msg += "0";
        msg += String(gps.date.year());
        msg += "#TIME=";
      #if GPS_FIX_TIME == 1
        if ((gps.time.hour() + 1) < 10)
          msg += "0";
        msg += (String(gps.time.hour() + 1) + ":");
      #else
        if (gps.time.hour() < 10)
          msg += "0";
        msg += (String(gps.time.hour()) + ":");
      #endif
        if (gps.time.minute() < 10)
          msg += "0";
        msg += String(gps.time.minute()) +  ":";
        msg += String(gps.time.second());
      }
    
    #if DEBUG_LORA_PACKET == 1
      Serial.println("Lora packet sent:");
      Serial.println(msg);
    #endif
      
      sendLoraPacket(msg);
    }
  }
}

// Function that builds a LoRa packet and sends it.
// The format of the LoRa packet is:
// | Destination |   Sender   |  Message ID  | Payload Length |  Payload data |
//    1 byte         1 byte       1 byte          1 byte           N byte
void sendLoraPacket(String data) {
  LoRa.beginPacket();              // start packet
  LoRa.write(destinationAddress);  // add destination address
  LoRa.write(localAddress);        // add sender address
  LoRa.write(msgCount);            // add message ID
  LoRa.write(data.length());       // add payload length
  LoRa.print(data);                // add payload
  LoRa.endPacket();                // finish packet and send it
  msgCount++;                      // increment message ID
}

// Function that reads data from the GPS module, parse it to get:
// latitude, longitude, altitude, time, date and other GPS values.
void updateGPSData() {
  unsigned long start = millis();
  do
  {
    while (GPSSerial.available())
      gps.encode(GPSSerial.read());
  } while (millis() - start < GPS_READ_TIME);

  if (gps.location.isValid()) {
  #if PRINT_GPS_DATA == 1
    SerialPort.print("Latitude: ");
    SerialPort.println(gps.location.lat(), 3);
    SerialPort.print("Longitude: ");
    SerialPort.println(gps.location.lng(), 3);
    if (gps.altitude.isValid()) {
      SerialPort.print("Altitude: ");
      SerialPort.println(gps.altitude.meters(), 3);
    }
  #endif
    GPSDataStatus[GPS_LOCATION] = true;
  }
  else {
  #if PRINT_GPS_DATA == 1
    SerialPort.println("Location: INVALID");
  #endif
    GPSDataStatus[GPS_LOCATION] = false;
  }

  if (gps.date.isValid()) {
  #if PRINT_GPS_DATA == 1
    SerialPort.print("Date: "); 
    SerialPort.print(gps.date.day());
    SerialPort.print("/");
    SerialPort.print(gps.date.month());
    SerialPort.print("/");
    SerialPort.println(gps.date.year());
  #endif

    GPSDataStatus[GPS_DATE] = true;
  }
  else {
#if PRINT_GPS_DATA == 1
    SerialPort.println("Date: INVALID");
#endif
    GPSDataStatus[GPS_DATE] = false;
  }

  if (gps.time.isValid()) {
#if PRINT_GPS_DATA == 1
    SerialPort.print("Time: ");
    if (gps.time.hour() < 10) SerialPort.print("0");
    SerialPort.print(gps.time.hour());
    SerialPort.print(":");
    if (gps.time.minute() < 10) SerialPort.print("0");
    SerialPort.print(gps.time.minute());
    SerialPort.print(":");
    if (gps.time.second() < 10) SerialPort.print("0");
    SerialPort.print(gps.time.second());
    SerialPort.print(".");
    if (gps.time.centisecond() < 10) SerialPort.print("0");
    SerialPort.println(gps.time.centisecond());
#endif
    GPSDataStatus[GPS_TIME] = true;
  }
  else {
#if PRINT_GPS_DATA == 1
    SerialPort.println("Time: INVALID");
#endif
    GPSDataStatus[GPS_TIME] = false;
  }
  
  if (gps.speed.isValid()) {
    SerialPort.print("Speed: ");
    SerialPort.print(gps.speed.knots());
    SerialPort.print(" (knots), ");
    SerialPort.print(gps.speed.mps());
    SerialPort.print(" m/s, ");
    SerialPort.print(gps.speed.kmph());
    SerialPort.println(" km/h");
  }

  if (gps.course.isValid()) {
    SerialPort.print("Course: ");
    SerialPort.println(gps.course.deg(), 2);
  }

  if (gps.satellites.isValid()) {
    SerialPort.print("Satellites: ");
    SerialPort.println(gps.satellites.value());
  }

  if(GPSDataStatus[GPS_LOCATION] && GPSDataStatus[GPS_DATE] && GPSDataStatus[GPS_TIME]) {
    isGPSDataValid = true;
  }
  else {
    isGPSDataValid = false;
  }
}
