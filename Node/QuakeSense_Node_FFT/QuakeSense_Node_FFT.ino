/*
    QuakeSense_Node_FFT

    QuakeSense is an IoT project that aims to "sense" and monitor
    earthquakes through the measures provided by the LSM6DSL accelerometer
    and a Lora network used to send alarms and environmental data.
    This is an improved version of QuakeSense_node that allows to
    - compute the FFT (Fast Fourier Transform) of the samples read from
      the accelerometer (experimental)
    - add a RX window to receive the ACK from the LoRa gateway (experimntal)

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
       the eartquake status, alarm messages and envirnmental data.

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
// Include Adafruit GPS library for MT3339 GPS module
#include <Adafruit_GPS.h>
// include SPI library and LoRa library
#include <SPI.h>
#include <LoRa.h>

#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

// Change these values according to your implementation
#define GET_ENV_DATA  1
#define PRINT_ENV_DATA 0
#define SEND_ENV_DATA 1
#define DEBUG_LORA_PACKET 1
#define READ_GYRO 0
#define PRINT_GYRO_DATA 0
#define PRINT_ACC_DATA  1
#define PRINT_GPS_DATA 1
#define SERIAL_DEBUG  0

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

#define PGHAX_THRESHOLD 70
#define PGHAY_THRESHOLD 70
#define PGVA_THRESHOLD 1100

#define DEV_I2C Wire

// Use Serial2 port (USART2 on PA1, PA0) to print data in the Serial Monitor
#define SerialPort Serial

// For GPS data use Serial6 port (USART6 on PA_12 (RX) and PA_11 (TX))
HardwareSerial GPSSerial(PA_12, PA_11);

Adafruit_GPS GPS(&GPSSerial);

// update gps data every 15 seconds
#define GPS_UPDATE_TIME 15000
uint32_t gps_timer;
bool isGPSDataValid = false;

const int csPin = 10;         // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

String data;    // outgoing message send through the LoRa module

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xAA;     // address of this device (transmitter)
byte destination = 0xBB;      // address of the receiver (LoRa gateway)

#define LORA_RX_INTERVAL 3000  // 3 seconds
bool loraInit = false;
bool ackReceived;

#if GET_ENV_DATA == 1
HTS221Sensor *HTS221_HumTemp;
LPS22HBSensor *LPS22HB_Press;
#endif
LSM6DSLSensor *LSM6DSL_AccGyro;

// Output data rate of LSM6DSL Accelerometer
// Available values: 13 Hz, 26 Hz, 52 Hz, 104 Hz, 208 Hz, 416 Hz,
//                   833 Hz, 1660 Hz, 3330 Hz, 6660 Hz
#define LSM6DSL_ACC_ODR LSM6DSL_ACC_GYRO_ODR_XL_104Hz
// Full scale range of LSM&DSL Accelerometer
// Available values: 2g, 4g, 8g, 16g
#define LSM6DSL_ACC_FS  LSM6DSL_ACC_GYRO_FS_XL_2g

const double samplingFrequency = 104;

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

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

volatile uint32_t counter = 0;

void setup() {
  // Initialize Serial communication at 115200 bps.
  SerialPort.begin(115200);
  // wait for the Serial port to open
  while (!SerialPort) ;

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

  // Set LSM6DSL Accelerometer full scale
  LSM6DSL_AccGyro->Set_X_FS(LSM6DSL_ACC_FS);
  // Set LSM6DSL Accelerometer output data rate
  LSM6DSL_AccGyro->Set_X_ODR(LSM6DSL_ACC_ODR);
  
#if READ_GYRO == 1
  LSM6DSL_AccGyro->Enable_G();
#endif

  // Enable free fall detection
  LSM6DSL_AccGyro->Enable_Free_Fall_Detection();
  // Enable tilt detection
  LSM6DSL_AccGyro->Enable_Tilt_Detection();
  // Enable wake up detection
  LSM6DSL_AccGyro->Enable_Wake_Up_Detection();

#if GET_ENV_DATA == 1
  env_data_timer = millis();
#endif

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
#if METHOD_TO_READ_SAMPLES == 2
  uint32_t quake_end_time = 0;
#endif
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
      else {
        quake_end_time = millis();
      }
    #endif
    }
  }
  
  if (n > 0) {
  
  #if METHOD_TO_READ_SAMPLES == 1
    duration = millis() - quake_start_time;
  #elif METHOD_TO_READ_SAMPLES == 2
    if(quake_end_time == 0) {
      quake_end_time = millis();
    }
    duration = quake_end_time - quake_start_time;
  #endif
  
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

    double vReal[3][MAX_SAMPLES];
    double vImag[3][MAX_SAMPLES] = {0};

    uint16_t i;
    for (i = 0; i < n; i++) {
      vReal[0][i] = (double) lsm6dsl_acc[i][LSM6DSL_X_AXIS];
      vReal[1][i] = (double) lsm6dsl_acc[i][LSM6DSL_Y_AXIS];
      vReal[2][i] = (double) lsm6dsl_acc[i][LSM6DSL_Z_AXIS];
    }
    while (i < MAX_SAMPLES) {
      vReal[0][i] = 0;
      vReal[1][i] = 0;
      vReal[2][i] = 0;

      i++;
    }

    double maxPeak[3];
    double fundFreq[3];
    
    for (i = 0; i < 3; i++) {
      
      if (i == 0) {
        SerialPort.println("X-AXIS:");
      }
      else if (i == 1) {
        SerialPort.println("Y-AXIS:");
      }
      else if (i == 2) {
        SerialPort.println("Z-AXIS:");
      }
    #if SERIAL_DEBUG == 1
      SerialPort.println("Accelerometer Data:");
      PrintVector(vReal[i], MAX_SAMPLES, SCL_TIME);
    #endif
          
      FFT.Windowing(vReal[i], MAX_SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Weigh data
    #if SERIAL_DEBUG == 1
      SerialPort.println("Weighed data:");
      PrintVector(vReal[i], MAX_SAMPLES, SCL_TIME);
    #endif
    
      FFT.Compute(vReal[i], vImag[i], MAX_SAMPLES, FFT_FORWARD); // Compute FFT
    #if SERIAL_DEBUG == 1
      SerialPort.println("Computed Real values:");
      PrintVector(vReal[i], MAX_SAMPLES, SCL_INDEX);
      SerialPort.println("Computed Imaginary values:");
      PrintVector(vImag[i], MAX_SAMPLES, SCL_INDEX);
    #endif
    
      fundFreq[i] = getFundamentalFreq(vReal[i], vImag[i], MAX_SAMPLES);
      FFT.ComplexToMagnitude(vReal[i], vImag[i], MAX_SAMPLES); // Compute magnitudes
    #if SERIAL_DEBUG == 1
      SerialPort.println("Computed magnitudes:");
      PrintVector(vReal[i], (MAX_SAMPLES >> 1), SCL_FREQUENCY);
    #endif
      SerialPort.print("Major Peak: ");
      maxPeak[i] = FFT.MajorPeak(vReal[i], MAX_SAMPLES, samplingFrequency);
      SerialPort.println(maxPeak[i], 3);
      SerialPort.print("Fundamental frequency: ");
      SerialPort.println(fundFreq[i], 3);
      SerialPort.println("--------------------------------------------------------");
    }

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

      // wait for the ACK sent by the gateway
      uint32_t timer = millis();
      SerialPort.println("Starting LoRa RX window");
      while ((millis() - timer) <= LORA_RX_INTERVAL) {
        // try to parse packet
        int packetSize = LoRa.parsePacket();
        ackReceived = false;

        if (packetSize) {
          // read bytes of packet header bytes:
          byte receiverAddr = LoRa.read();    // receiver address
          byte senderAddr = LoRa.read();      // sender address
          byte msgId = LoRa.read();
          byte msgLength = LoRa.read();       // incoming message (ACK) length

          String data = "";

          // read payload of the packet
          while (LoRa.available()) {
            data += (char)LoRa.read();
          }

          // check length for error
          if ((msgLength == data.length()) && (receiverAddr == localAddress)) {
#if DEBUG_LORA_PACKET == 1
            // if message is for this device, print details and get the msg fields:
            SerialPort.println("Packet Info:");
            SerialPort.println("Received from: 0x" + String(senderAddr, HEX));
            SerialPort.println("Sent to: 0x" + String(receiverAddr, HEX));
            SerialPort.println("Message length: " + String(msgLength));
            SerialPort.println("RSSI: " + String(LoRa.packetRssi()));
            SerialPort.println("SNR: " + String(LoRa.packetSnr()));
            SerialPort.println("\nMessage Data:");
            SerialPort.println(data);
            SerialPort.println();
#endif
            if (data.equals("#ACK")) {
              SerialPort.println("ACK received: LoRa gateway has successfully sent the message to Adafruit IO");
              ackReceived = true;
            }
          }
        }
      }
      SerialPort.println("End of LoRa RX window");
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

void PrintVector(double * vData, uint16_t bufferSize, uint8_t scaleType) {
  for (uint16_t i = 0; i < bufferSize; i++) {
    double abscissa;
    // Print abscissa value
    switch (scaleType) {
      case SCL_INDEX:
        abscissa = (i * 1.0);
        break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
        break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / MAX_SAMPLES);
        break;
    }
    Serial.print(abscissa, 6);
    Serial.print(" ");
    Serial.print(vData[i], 4);
    Serial.println();
  }
  Serial.println();
}

double getFundamentalFreq(double * vReal, double * vImag, uint16_t samples) {
  double frequencies[samples];
  double fundamentalFreq;

  for (uint16_t i = 0; i < samples; i++) {
    frequencies[i] = (atan(vImag[i] / vReal[i]) / (2 * PI));
    if (i == 0) {
      fundamentalFreq = frequencies[i];
    }
    else {
      if (fundamentalFreq < frequencies[i]) {
        fundamentalFreq = frequencies[i];
      }
    }
  }

  return fundamentalFreq;
}
