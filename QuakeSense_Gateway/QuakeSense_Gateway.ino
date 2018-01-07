/* 
 *  QuakeSense_Gateway
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
 *     - a development board that is used to control the accelerometer, 
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
 *     the earthquake status, alarm messages and envirnmental data.
 *     
 *     In this implementation, AdafruitIO has been choosen as the
 *     cloud platform and data coming from the LoRa gateway is sent 
 *     to Adafruit IO using the MQTT protocol
 *     
 *  Copyright (C) Biagio Montaruli <biagio.hkr@gmail.com>
 *  
 *  This program is free software: you can redistribute it and/or modify 
 *  it under the terms of the GNU General Public License as published by 
 *  the Free Software Foundation, either version 3 of the License, or 
 *  (at your option) any later version.
 *
 */

// Include libraries
#include <SPI.h>
#include <WiFiST.h>
#include <WiFiServerST.h>
#include <WiFiClientST.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <LoRa.h>

SPIClass SPI_3(PC12, PC11, PC10);
WiFiClass WiFi(&SPI_3, PE0, PE1, PE8, PB13);

/************************* WiFi Configuration *********************************/
// Update these values according to your WiFi network
#define WLAN_SSID       ""
#define WLAN_PASS       ""

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883            // use 8883 for SSL/TLS
#define AIO_USERNAME    ""
#define AIO_KEY         ""

/******************************* MQTT *************************************/

// Create a new object of WiFiClient class to connect to the MQTT server.
WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
// Setup a feed called 'hts221-temp' for publishing.
Adafruit_MQTT_Publish tempFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hts221-temp");

// Setup a feed called 'hts221-hum' for publishing.
Adafruit_MQTT_Publish humFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hts221-humidity");

// Setup a feed called 'lps22hb-pressure' for publishing.
Adafruit_MQTT_Publish pressFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/lps22hb-pressure");

// Setup a feed called 'earthquake-alert' for publishing.
Adafruit_MQTT_Publish alertFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/earthquake-alert");

// Setup a feed called 'pgha-x' for publishing.
Adafruit_MQTT_Publish pghaxFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pgha-x");

// Setup a feed called 'pgha-y' for publishing.
Adafruit_MQTT_Publish pghayFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pgha-y");

// Setup a feed called 'pgva' for publishing.
Adafruit_MQTT_Publish pgvaFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pgva");

/************************ LoRa Config **************************************/
const int csPin = 10;         // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

byte msgCount = 0;                // count of outgoing messages
byte localAddress = 0xBB;         // address of this device
byte destinationAddress = 0xAA;   // destination to send to

/*********************** Serial Config *************************************/
#define SerialPort Serial
#define SERIAL_DEBUG 1

/*************************** Sketch Code ************************************/
void setup() {
  // Initialize Serial communication at 115200 bps.
  SerialPort.begin(115200);
  // wait for the Serial port to open
  // while(!SerialPort) ;
  delay(10);

  SerialPort.println("B-L475E-IOT01A Discovery IoT Node - LoRa Gateway");

  // set CS, reset, IRQ pin for SX1276 LoRa module
  LoRa.setPins(csPin, resetPin, irqPin);

  // Set LoRa frequency band to 868 MHz
  if (!LoRa.begin(868E6)) {
    SerialPort.println("LoRa init failed. Check your connections.");
  }
  SerialPort.println("LoRa radio successfully initialized.");

  // Set LoRa Mode 3
  // BW = 125 kHz; CR = 4/5; SF = 10
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  // register the receive callback
  LoRa.onReceive(parseLoRaPacket);

  // put the radio into receive mode
  LoRa.receive();

  // Initialize the WiFi module
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi Module ISM43362-M3G-L44 not present");
    while (true) ;
  }

  // Print firmware version
  String fv = WiFi.firmwareVersion();
  Serial.print("Firwmare version: ");
  Serial.println(fv);

  // Attempt to connect to Wifi network
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(WLAN_SSID);

    // Connect to WPA/WPA2 network.
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    // wait 10 seconds for connection:
    delay(10000);
  }
  printWifiStatus();
}

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).
  // See the MQTT_connect function definition further below.
  MQTT_connect();

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  // if(!mqtt.ping()) {
  //   mqtt.disconnect();
  // }
}

// Function that parse a LoRa packet sent by a LoRa Node
// The format of the LoRa packet is:
// |  Receiver Addr  |  Sender Addr  |  Message ID  |  Payload Length  |  Payload data  |
//       1 byte           1 byte          1 byte          1 byte            N byte
void parseLoRaPacket(int packetSize) {
  // if there's no data, return
  if (packetSize == 0) return;

  // read bytes of packet header bytes:
  byte receiverAddr = LoRa.read();    // receiver address
  byte senderAddr = LoRa.read();      // sender address
  byte msgId = LoRa.read();       // incoming message ID
  byte msgLength = LoRa.read();   // incoming message length

  String data = "";

  // read payload of the packet
  while (LoRa.available()) {
    data += (char)LoRa.read();
  }

  // check length for error
  if (msgLength != data.length()) {
    SerialPort.println("LoRa Error: wrong message length");
    // skip rest of function
    return;
  }

  // if the receiver isn't this device discard the packet
  if (receiverAddr != localAddress) {
    SerialPort.print("Received a packet with an unknow address: ");
    SerialPort.print(receiverAddr, HEX);
    SerialPort.println("\nPacket will be discarded.");
    // skip rest of function
    return;
  }

  // if message is for this device, print details and get the msg fields:
  SerialPort.println("Packet Info:");
  SerialPort.println("Received from: 0x" + String(senderAddr, HEX));
  SerialPort.println("Sent to: 0x" + String(receiverAddr, HEX));
  SerialPort.println("Message ID: " + String(msgId));
  SerialPort.println("Message length: " + String(msgLength));
  SerialPort.println("RSSI: " + String(LoRa.packetRssi()));
  SerialPort.println("Snr: " + String(LoRa.packetSnr()));
  SerialPort.println("\nMessage Data:");
  SerialPort.println(data);
  SerialPort.println();

  int startIndex, stopIndex;
  String tempData = "";
  String alertMsg = "EARTHQUAKE ALERT!!\n";
  
  // A LoRa node can send two types of messages:
  // 1) Messages containing environmental data which have the following format:
  //    #TEMP=<temperature>#HUM=<humidity>#PRESS=<pressure>
  // 2) Eartquake alert messages which have the following formats:
  //    - Full format with GPS data
  //      #ALERT!#PGHAX=<pgha x-axis>#PGHAY=<pgha y-axis>#PGVA=<pgva>#DUR=<duration>#LAT=<latitude>#LON=<longitude>#ALT=<altitude>#DATE=<date>#TIME=<time>
  //    - Reduced format without GPS data
  //      #ALERT!#PGHAX=<pgha x-axis>#PGHAY=<pgha y-axis>#PGVA=<pgva>#DUR=<duration>

  // Check if the received LoRa packet contains an alert message
  if (data.startsWith("#ALERT!")) {
    // send alert to Adafruit IO:
    if ((data.indexOf("#PGHAX=") != -1) && (data.indexOf("#PGHAY=") != -1) && (data.indexOf("#PGVA=") != -1)) {
      startIndex = data.indexOf("#PGHAX=") + String("#PGHAX=").length();
      stopIndex = data.indexOf("#PGHAY=");
      tempData = data.substring(startIndex, stopIndex);
    #if SERIAL_DEBUG == 1
      Serial.print("PGHAX = "); Serial.println(tempData);
    #endif
      pghaxFeed.publish(tempData.toInt());

      startIndex = stopIndex + String("#PGHAY=").length();
      stopIndex = data.indexOf("#PGVA=");
      tempData = data.substring(startIndex, stopIndex);
    #if SERIAL_DEBUG == 1
      Serial.print("PGHAY = "); Serial.println(tempData);
    #endif
      pghayFeed.publish(tempData.toInt());

      startIndex = stopIndex + String("#PGVA=").length();
      stopIndex = data.indexOf("#DUR=");
      tempData = data.substring(startIndex, stopIndex);
    #if SERIAL_DEBUG == 1
      Serial.print("PGVA = "); Serial.println(tempData);
    #endif
      pgvaFeed.publish(tempData.toInt());

      // full format earthquake aler message
      if (data.indexOf("#LAT=") != -1) {
        startIndex = stopIndex + String("#DUR=").length();
        stopIndex = data.indexOf("#LAT=");
        tempData = data.substring(startIndex, stopIndex);
      #if SERIAL_DEBUG == 1
        Serial.print("DUR = "); Serial.println(tempData);
      #endif
        alertMsg += ("Duration: " + tempData + "\n");

        startIndex = stopIndex + String("#LAT=").length();
        stopIndex = data.indexOf("#LON=");
        tempData = data.substring(startIndex, stopIndex);
      #if SERIAL_DEBUG == 1
        Serial.print("LAT = "); Serial.println(tempData);
      #endif
        alertMsg += ("Latitude: " + tempData + "\n");

        startIndex = stopIndex + String("#LON=").length();
        stopIndex = data.indexOf("#ALT=");
        tempData = data.substring(startIndex, stopIndex);
      #if SERIAL_DEBUG == 1
        Serial.print("LON = "); Serial.println(tempData);
      #endif
        alertMsg += ("Longitude: " + tempData + "\n");

        startIndex = stopIndex + String("#ALT=").length();
        stopIndex = data.indexOf("#DATE=");
        tempData = data.substring(startIndex, stopIndex);
      #if SERIAL_DEBUG == 1
        Serial.print("ALT = "); Serial.println(tempData);
      #endif
        alertMsg += ("Altitude: " + tempData + "\n");

        startIndex = stopIndex + String("#DATE=").length();
        stopIndex = data.indexOf("#TIME=");
        tempData = data.substring(startIndex, stopIndex);
      #if SERIAL_DEBUG == 1
        Serial.print("DATE = "); Serial.println(tempData);
      #endif
        alertMsg += ("Date: " + tempData + "\n");

        startIndex = stopIndex + String("#TIME=").length();
        // stopIndex = data.length();
        // tempData = data.substring(startIndex, stopIndex);
        tempData = data.substring(startIndex);
      #if SERIAL_DEBUG == 1
        Serial.print("Time = "); Serial.println(tempData);
      #endif      
        alertMsg += ("Time: " + tempData);
        SerialPort.print("Earthquake Alert Message: "); SerialPort.println(alertMsg);
        alertFeed.publish(alertMsg.c_str());
      }
      // reduced format earthquake alert message
      else {
        startIndex = stopIndex + String("#DUR=").length();
        tempData = data.substring(startIndex);
      #if SERIAL_DEBUG == 1
        Serial.print("DUR = "); Serial.println(tempData);
      #endif
        alertMsg += ("Duration: " + tempData + "\n\n");
        SerialPort.println("Earthquake Alert Message:"); SerialPort.println(alertMsg);
        alertFeed.publish(alertMsg.c_str());
      }
    }
  }
  // message containing environmental data
  else if (data.startsWith("#TEMP=") && (data.indexOf("#HUM=") != -1) && (data.indexOf("#PRESS=") != -1)) {
    startIndex = data.indexOf("#TEMP=") + String("#TEMP=").length();
    stopIndex = data.indexOf("#HUM=");
    tempData = data.substring(startIndex, stopIndex);
  #if SERIAL_DEBUG == 1
    Serial.print("TEMP = "); Serial.println(tempData);
  #endif
    tempFeed.publish(tempData.toFloat());

    startIndex = stopIndex + String("#HUM=").length();
    stopIndex = data.indexOf("#PRESS=");
    tempData = data.substring(startIndex, stopIndex);
  #if SERIAL_DEBUG == 1
    Serial.print("HUM = "); Serial.println(tempData);
  #endif
    humFeed.publish(tempData.toFloat());

    startIndex = stopIndex + String("#PRESS=").length();
    tempData = data.substring(startIndex);
  #if SERIAL_DEBUG == 1
    Serial.print("PRESS = "); Serial.println(tempData);
  #endif
    pressFeed.publish(tempData.toFloat());
  }

  SerialPort.println("------------------------------------------");
}

// Function to connect and reconnect as necessary to the MQTT server (Adafruit IO MQTT Broker).
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  SerialPort.println("Connecting to Adafruit IO through MQTT...");

  uint8_t retries = 3;
  // connect will return 0 for connected
  while ((ret = mqtt.connect()) != 0) {
    SerialPort.println(mqtt.connectErrorString(ret));
    SerialPort.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      SerialPort.println("Failed connecting to Adafruit IO through MQTT");
      while (true) ;
    }
  }
  SerialPort.println("Connected!");
}

// Function that prints WiFi status and connection parameters.
void printWifiStatus() {
  // print the SSID of the network you're connected to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the BSSID of the network you're connected to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(bssid[i], HEX);
    if (i != 5) {
      Serial.print(":");
    }
    else {
      Serial.println();
    }
  }

  // print the MAC address of the WiFi module:
  byte macAddr[6];
  WiFi.macAddress(macAddr);
  Serial.print("MAC Address: ");
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(macAddr[i], HEX);
    if (i != 5) {
      Serial.print(":");
    }
    else {
      Serial.println();
    }
  }

  // print IP address of the WiFi module assigned through DHCP:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the subnet mask of the WiFi network you're connected to:
  IPAddress subMask = WiFi.subnetMask();
  Serial.print("IP Address: ");
  Serial.println(subMask);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
