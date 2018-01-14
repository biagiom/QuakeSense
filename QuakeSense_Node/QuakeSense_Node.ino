/* 
 *  QuakeSense_Node
 *  
 *  QuakeSense is an IoT project that aims to "sense" and monitor
 *  earthquakes through the measures provided by the LSM6DSL accelerometer
 *  and a Lora network used to send alarms and environmental data.
 *  
 *  The main elements of this project are:
 *  1) At least one sensor node consisting of:
 *     - a 3-axis accelerometer used to measure seismic waves result
 *       in the strong ground motion.
 *     - a LoRa radio module used to transmit an alarm and data.
 *       coming from the accelermoter to a single channel LoRa gateway
 *     - A GPS module used to get altitude, longitude and latitude of
 *       the position of the seismic event.
 *     - a development board used to control the accelerometer, 
 *       the LoRa module and the GPS module.
 *     - a LiPo battery used to power the development board.
 *     
 *     In this implementation, the sensor node has been made with:
 *     > LSM6DSL - 3D accelerometer and 3D gyroscope
 *     > STM32 Nucleo F401RE as the development board
 *     > Dragino LoRa/GPS Shield including
 *       - a 137 MHz to 1020 MHz Low Power Long Range LoRa RF Transceiver
 *       - a L80 GPS module based on MTK MT3339
 *  
 *  2) A LoRa gateway used to receive environmental data and messages
 *     from the sensor nodes and to send them to a cloud platform.
 *     
 *     In this implementation, the LoRa gateway has been made with:
 *     > B-L475E-IOT01A2 STM32L4 Discovery kit IoT node featuring
 *       - Wi-Fi® module Inventek ISM43362-M3G-L44 (802.11.4 b/g/n compliant)
 *       - Sub-GHz (868 Mhz) low-power RF module SPSGRF-868
 *       - Bluetooth® V4.1 module (SPBTLE-RF)
 *       - Capacitive humidity and temperature sensor (HTS221)
 *       - 3-axis magnetometer (LIS3MDL)
 *       - 3D accelerometer and 3D gyroscope (LSM6DSL)
 *       - 260-1260 hPa absolute digital output barometer (LPS22HB) 
 *     > Dragino LoRa Shield which includes:
 *       - the SX1276 868 Mhz Low Power Long Range LoRa RF Transceiver
 *     
 *  3) A cloud platform in order to allow the user to visualize 
 *     the eartquake status, alarm messages and envirnmental data.
 *     
 *     In this implementation, Adafruit IO has been choosen as the
 *     cloud platform and data coming from the LoRa gateway is sent
 *     to Adafruit IO using the MQTT protocol.
 *     
 *  Copyright (C) Biagio Montaruli <biagio.hkr@gmail.com>
 *  
 *  This program is free software: you can redistribute it and/or modify 
 *  it under the terms of the GNU General Public License as published by 
 *  the Free Software Foundation, either version 3 of the License, or 
 *  (at your option) any later version.
 *
 */

// Include sensor's library
#include <HTS221Sensor.h>
#include <LPS22HBSensor.h>
#include <LSM6DSLSensor.h>
// Include Adafruit GPS library for MT3339 GPS module
#include <Adafruit_GPS.h>
// include SPI library and LoRa library
#include <SPI.h>
#include <LoRa.h>

// Change the values according to your implementation
#define GET_ENV_DATA  1
#define PRINT_ENV_DATA 1
#define SEND_ENV_DATA 1
#define DEBUG_LORA_PACKET 1
#define READ_GYRO 0
#define PRINT_GYRO_DATA 0
#define PRINT_ACC_DATA  1
#define PRINT_GPS_DATA 1
#define SERIAL_DEBUG  0

#define DEV_I2C Wire

// Use Serial2 port (USART2 on PA1, PA0) to print data in the Serial Monitor
#define SerialPort Serial

// For GPS data use Serial6 port (USART6 on PA_12 (RX) and PA_11 (TX))
HardwareSerial GPSSerial(PA_12, PA_11);

Adafruit_GPS GPS(&GPSSerial);

// update gps data every 5 minutes (300000 millis)
#define GPS_UPDATE_TIME 300000
uint32_t gps_timer;
bool isGPSDataValid = false;

const int csPin = 10;         // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

String data;    // outgoing message send through the LoRa module

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xAA;     // address of this device (transmitter)
byte destination = 0xBB;      // address of the receiver (LoRa gateway)

bool loraInit = false;

// Object that represents sensors
#if GET_ENV_DATA == 1
HTS221Sensor *HTS221_HumTemp;
LPS22HBSensor *LPS22HB_Press;
#endif
LSM6DSLSensor *LSM6DSL_AccGyro;

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

uint32_t current_time;

#if GET_ENV_DATA == 1
// time interval to update the environmental data:
// temperature humidity and pressure
#define ENVDATA_UPDATE_TIME 10000
uint32_t env_data_timer;
#endif

// Maximum number of samples read by the accelerometer
#define MAX_SAMPLES 128

volatile uint32_t counter = 0;

void setup() {
  // Initialize Serial communication at 115200 bps.
  SerialPort.begin(115200);
  // wait for the Serial port to open
  // while(!SerialPort) ;

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Enable interrupts for LSM6DSL accelerometer sensor
  attachInterrupt(D4, event_cb, RISING);
  attachInterrupt(D5, event_cb, RISING);

  // Enable and initialize sensors.
#if GET_ENV_DATA == 1
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
#if READ_GYRO == 1
  LSM6DSL_AccGyro->Enable_G();
#endif

  // Enable free fall detection
  LSM6DSL_AccGyro->Enable_Free_Fall_Detection();
  // Enable tilt detection
  LSM6DSL_AccGyro->Enable_Tilt_Detection();
  // Enable wake up detection
  LSM6DSL_AccGyro->Enable_Wake_Up_Detection();

  env_data_timer = millis();

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
  GPS.begin(9600);

  gps_timer = millis();

  // Turn on RMC and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate for the GPS Module to 1 Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // Request updates on antenna status
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  updateGPSData();
}

void loop() {
  current_time = millis();

#if GET_ENV_DATA == 1
  // Check if the environmental data needs to be updated
  if ((current_time - env_data_timer) >= ENVDATA_UPDATE_TIME) {
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
#if PRINT_ENV_DATA ==1
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
#endif
    sendLoraPacket(msg);
#endif

    env_data_timer = millis();
  }
#endif

  // Check if the GPS data needs to be updated.
  if ((current_time - gps_timer) >= GPS_UPDATE_TIME) {
    updateGPSData();
    gps_timer = millis();
  }
}

void earthquake_detection() {
  uint32_t quake_start_time = 0;
  uint32_t duration = 0;
  
  // Read accelerometer and gyroscope LSM6DSL.
  int32_t lsm6dsl_acc[MAX_SAMPLES][LSM6DSL_NUM_AXIS];
#if READ_GYRO == 1
  int32_t lsm6dsl_gyro[MAX_SAMPLES][LSM6DSL_GYRO_NUM_DATA];
#endif

  uint16_t n;

  for (n = 0; n < MAX_SAMPLES; n++) {
    LSM6DSL_AccGyro->Get_X_Axes(lsm6dsl_acc[n]);
    if ( (abs(lsm6dsl_acc[n][LSM6DSL_X_AXIS]) > 80) || (abs(lsm6dsl_acc[n][LSM6DSL_Y_AXIS]) > 80) || (abs(lsm6dsl_acc[n][LSM6DSL_Z_AXIS]) > 1080) ) {
      if (n == 0) {
        quake_start_time = millis();
      }
#if READ_GYRO == 1
      LSM6DSL_AccGyro->Get_G_Axes(lsm6dsl_gyro[n]);
#endif
    }
    else {
      break;
    }
  }

  if (n > 0) {
    duration = millis() - quake_start_time;
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

    SerialPort.print("Earthquake duration: ");
    SerialPort.print(duration);
    SerialPort.println(" msecs");

    // calculate peak ground horizontal acceleration
    // PGHA for x-axis and y-axis
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

    SerialPort.print("PGHA (x-axis): ");
    SerialPort.println(pgha_x);
    SerialPort.print("PGHA (y-axis): ");
    SerialPort.println(pgha_y);
    SerialPort.print("PGVA (z-axis): ");
    SerialPort.println(pgva);

#if SERIAL_DEBUG == 1
    SerialPort.print("Event N° ");
    SerialPort.println(counter);

    SerialPort.print("Got ");
    SerialPort.print(n);
    SerialPort.println(" samples.");
#endif

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
        msg += String(GPS.latitude, 2);
        msg += "#LON=";
        msg += String(GPS.longitude, 2);
        msg += "#ALT=";
        msg += String(GPS.altitude, 2);
        msg += "#DATE=";
        msg += (String(GPS.day) + "/" + String(GPS.month) + "/20" + String(GPS.year));
        msg += "#TIME=";
        msg += (String(GPS.hour) + ":" + String(GPS.minute) +  ":" + String(GPS.seconds));
      }
#ifdef DEBUG_LORA_PACKET
      Serial.println("Lora packet sent:");
      Serial.println(msg);
#endif
      sendLoraPacket(msg);
    }
  }
}

void event_cb() {
  LSM6DSL_Event_Status_t status;
  LSM6DSL_AccGyro->Get_Event_Status(&status);

  counter++;

  if (status.WakeUpStatus) {
    SerialPort.println("Motion Detected!");
    earthquake_detection();
  }

  if (status.FreeFallStatus) {
    SerialPort.println("Free Fall Detected!");
  }

  if (status.TiltStatus) {
    SerialPort.println("Tilt Detected!");
  }
}

// Function that builds a LoRa packet and sends it.
// The format of the LoRa packet is:
// | Destination |   Sender   |  Message ID  | Payload Length |  Payload data |
//    1 byte         1 byte       1 byte          1 byte           N byte
void sendLoraPacket(String data) {
  LoRa.beginPacket();             // start packet
  LoRa.write(destination);        // add destination address
  LoRa.write(localAddress);       // add sender address
  LoRa.write(msgCount);           // add message ID
  LoRa.write(data.length());      // add payload length
  LoRa.print(data);               // add payload
  LoRa.endPacket();               // finish packet and send it
  msgCount++;                     // increment message ID
}

// Function that reads data from the GPS module, parse it to get:
// latitude, longitude, altitude, time, date and other GPS values.
void updateGPSData() {
  uint32_t timer = millis();

  bool NMEAparsed = false;
  while (millis() - timer < 2000) {
    // read data from the GPS
    char c = GPS.read();

    // print GPS data in the Serial monitor
   #if SERIAL_DEBUG == 1
    if (c) SerialPort.print(c);
  #endif

    if (GPS.newNMEAreceived()) {
      // this also sets the newNMEAreceived() flag to false
      // we can fail to parse a sentence in which case we should just wait for another
      if (GPS.parse(GPS.lastNMEA())) {
        NMEAparsed = true;
        break;
      }
    }
  }

  if (NMEAparsed == true) {

#if SERIAL_DEBUG == 1
    // if a sentence is received, we can check the checksum and parse it...
    // this also sets the newNMEAreceived() flag to false
    SerialPort.println(GPS.lastNMEA());
#endif

#if PRINT_GPS_DATA == 1
    SerialPort.print("\nTime: ");
    SerialPort.print(GPS.hour, DEC); SerialPort.print(':');
    SerialPort.print(GPS.minute, DEC); SerialPort.print(':');
    SerialPort.print(GPS.seconds, DEC); SerialPort.print('.');
    SerialPort.println(GPS.milliseconds);
    SerialPort.print("Date: ");
    SerialPort.print(GPS.day, DEC); SerialPort.print('/');
    SerialPort.print(GPS.month, DEC); SerialPort.print("/20");
    SerialPort.println(GPS.year, DEC);
    SerialPort.print("Fix: "); SerialPort.print((int)GPS.fix);
    SerialPort.print(" quality: "); SerialPort.println((int)GPS.fixquality);
#endif
    if (GPS.fix) {

#if PRINT_GPS_DATA == 1
      SerialPort.print("Location: ");
      SerialPort.print(GPS.latitude, 4); SerialPort.print(GPS.lat);
      SerialPort.print(", ");
      SerialPort.print(GPS.longitude, 4); SerialPort.println(GPS.lon);
      SerialPort.print("Speed (knots): "); SerialPort.println(GPS.speed);
      SerialPort.print("Angle: "); SerialPort.println(GPS.angle);
      SerialPort.print("Altitude: "); SerialPort.println(GPS.altitude);
      SerialPort.print("Satellites: "); SerialPort.println((int)GPS.satellites);
#endif

      isGPSDataValid = true;
    }
    else {
      isGPSDataValid = false;
    }
  }
}
