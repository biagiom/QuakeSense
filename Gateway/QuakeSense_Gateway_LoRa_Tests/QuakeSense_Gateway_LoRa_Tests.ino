/*
 *  QuakeSense_Gateway_LoRa_Tests
 * 
 *  QuakeSense is an IoT project that aims to "sense" and monitor earthquakes through a LoRa network.
 * 
 *  The main elements of this project are:
 *  1) At least one Sensor Node (Mote) consisting of:
 *     - a 3-axis accelerometer used to measure strong ground motion activity.
 *     - a LoRa radio module used to transmit an alarm and the main parameters that
 *       characterize strong motion activity to a single channel LoRa gateway
 *     - A GPS module used to get location (altitude, longitude and latitude)
 *       and date relative to the seismic event.
 *     - a development board that is used to control the accelerometer,
 *       the LoRa module and the GPS module.
 *     - a LiPo battery used to power the development board.
 *     - a solar panel used to charge the LiPo battery.
 *
 *     In this implementation, the sensor node has been made with:
 *     > X-NUCLEO-IKS01A2 motion MEMS and environmental sensor expansion board including
 *       * LSM6DSL: MEMS 3D accelerometer and 3D gyroscope
 *       * LSM303AGR: MEMS 3D accelerometer and magnetometer
 *       * LPS22HB: MEMS pressure sensor
 *       * HTS221: capacitive digital relative humidity and temperature sensor
 *     > STM32 Nucleo F401RE as the development board
 *     > Dragino LoRa/GPS Shield including
 *       - RFM95W 137 MHz to 1020 MHz low-power, long-range LoRa RF transceiver
 *       - Quectel L80 GPS module based on MTK MT3339
 *     > Seed Studio Solar Charger shield v2.2 to which are connected
 *       a 2000 mAh LiPo battery and a 1.5 W solar panel
 *
 *     In presence of a seismic event, the STM32 Nucleo board reads acceleration samples from
 *     the LSM6DSL sensor to calculate some of the main parameters that characterize the 
 *     strong-motion activity: bracketed duration and peak ground acceleration 
 *     relative to the three components of the motion (x, y and z).
 *     Then the Sensor Node sends the calculated parameters to the LoRa gateway.
 *     Each sensor node uses periodically the HTS221 and LPS22HB environmental sensors to get
 *     temperature, relative humidity and pressure and sends these values to the gateway.
 *
 *  2) A LoRa gateway that receives environmental data and earthquake alert messages
 *     from the sensor nodes and sends them to a IoT Platform.
 *
 *     In this implementation, the LoRa gateway has been made with:
 *     > B-L475E-IOT01A2 STM32L4 Discovery kit IoT node featuring:
 *       - Wi-Fi module Inventek ISM43362-M3G-L44 (802.11b/g/n)
 *       - SPSGRF-868: Sub-GHz (868 Mhz) low-power RF module
 *       - SPBTLE-RF: Bluetooth V4.1 module
 *       - HTS221: capacitive relative humidity and temperature sensor
 *       - LSM303AGR: MEMS 3D accelerometer and MEMS 3D magnetometer
 *       - LSM6DSL: MEMS 3D accelerometer and MEMS 3D gyroscope
 *       - LSP22HB: 260-1260 hPa absolute digital output barometer
 *     > Dragino LoRa Shield which includes:
 *       - the RFM95W low-power, long-range LoRa RF transceiver based on SX1276
 *
 *  3) A cloud platform that allows the user to visualize earthquakes' parameters,
 *     alarm messages and envirnomental data in real-time.
 *     In the following implementation, AdafruitIO has been choosen as the cloud platform
 *     and data coming from the LoRa gateway is sent to Adafruit IO using the MQTT protocol.
 *
 *  The following implementation allows also the gateway to send the acknowledge to a sensor node
 *  to notify it about the sending of the data to Adafruit IO Platform.
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

/***************************************** WiFi Configuration *******************************************/
// Update these values according to your WiFi network
#define WLAN_SSID       "your_wifi_ssid"
#define WLAN_PASS       "your_wifi_password"

/******************************************* Adafruit.IO Setup ******************************************/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883  // use 8883 and WiFiClientSecure (if supported) for SSL/TLS
// Update the following values according to your Adafruit IO account
#define AIO_USERNAME    "your_aio_username"
#define AIO_KEY         "your_aio_key"

/********************************************** MQTT ****************************************************/
// Create a new object of WiFiClient class to connect to the MQTT server.
WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/******************************************* Feeds ******************************************************/
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
// Setup a feed called 'lora-packet-info' for publishing.
Adafruit_MQTT_Publish packetInfoFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/lora-packet-info", MQTT_QOS_1);

Adafruit_MQTT_Publish packetCounterFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/lora-packet-counter", MQTT_QOS_1);

Adafruit_MQTT_Subscribe loraSettingsFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/lora-settings", MQTT_QOS_1);

String publishMsg;

/*************************************** LoRa Config **************************************************/
const int csPin = 10;         // LoRa radio chip select
const int resetPin = 5;       // LoRa radio reset
const int irqPin = 3;         // change for your board; must be a hardware interrupt pin

byte counterMsg = 0;          // counter of received messages having the same ID
byte gatewayAddress = 0xBB;   // address of this device (LoRa Gateway)
byte nodeAddress;             // address of LoRa node to send ACK

byte lastMsgID = 0;           // ID of the last received LoRa packet

bool sendACK = false;
byte msgIdACK;

#define LORA_HEADER_LEN 4

/***************************************** Serial Config *********************************************/
#define SerialPort Serial
#define SERIAL_DEBUG 1

/***************************************** Sketch Code **********************************************/
void setup() {
  // Initialize serial communication at 115200 bps:
  SerialPort.begin(115200);
  // wait for the Serial port to open
  while (!SerialPort);
  delay(10);

  SerialPort.println("QuakeSense LoRa Gateway");

  // Set CS, reset, IRQ pin for SX1276 LoRa module
  LoRa.setPins(csPin, resetPin, irqPin);

  // Set LoRa frequency band to 868 MHz
  if (!LoRa.begin(868E6)) {
    SerialPort.println("LoRa init failed. Check your connections.");
  }
  SerialPort.println("LoRa module successfully initialized.");

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
    Serial.println("WiFi Module ISM43362-M3G-L44 not detected");
    while (true);
  }

  // Print firmware version
  String fv = WiFi.firmwareVersion();
  Serial.print("Firwmare version: ");
  Serial.println(fv);

  // Attempt to connect to Wifi network
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to the WiFi network ");
    Serial.println(WLAN_SSID);

    // Connect to WPA/WPA2 network.
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    // wait 8 seconds for connection:
    delay(8000);
  }
  SerialPort.println("WiFi connected:");
  printWifiStatus();

  loraSettingsFeed.setCallback(loraSettingsCallback);
  mqtt.subscribe(&loraSettingsFeed);
}

void loop() {

  if (sendACK == true) {
    // Send ACK to LoRa Node:
    // alert message, earthquake and environmental data have been sent to Adafruit IO
    sendLoraACKPacket();
    sendACK = false;
    SerialPort.println("ACK sent to LoRa Node 0x" + String(nodeAddress, HEX));
    Serial.println();
    // wait 500 millis and go back into receive mode
    delay(500);
    LoRa.receive();

    packetCounterFeed.publish((uint32_t) counterMsg);

    delay(500);
  
    packetInfoFeed.publish(publishMsg.c_str());

    delay(500);

  #if SERIAL_DEBUG == 1
    SerialPort.print("Message published: "); SerialPort.println(publishMsg);
  #endif
  }
  
  // Ensure the connection to the MQTT server is alive:
  // this will make the first connection and automatically reconnect when disconnected.
  // See the MQTT_connect function definition further below.
  MQTT_connect();

  mqtt.processPackets(10000);

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  //  if (!mqtt.ping()) {
  //     mqtt.disconnect();
  //  }
}

byte getChecksum(byte *buffer, uint16_t length) {
  byte checksum = 0;
  
  for (uint32_t i = 0; i < length; i++) {
    if ((checksum + buffer[i]) > 255) {
      checksum += (255 - buffer[i]);
    }
    else {
      checksum += buffer[i];
    }
  }

  return checksum;
}

// Function that builds a LoRa ACK packet and sends it.
// The format of the LoRa packet is:
// |  Receiver Addr  |  Sender Addr  |  Message ID  |  Payload Length  |  Payload data  |  Checksum  |
//       1 byte           1 byte          1 byte          1 byte            N byte          1 byte
void sendLoraACKPacket() {
  String payload = "#ACK";

  LoRa.beginPacket();              // start packet
  LoRa.write(nodeAddress);         // add destination (LoRa node) address
  LoRa.write(gatewayAddress);      // add sender (LoRa gateway) address
  LoRa.write(msgIdACK);
  LoRa.write(payload.length());    // add payload length
  uint16_t payloadLen = payload.length();
  byte buf[payloadLen + LORA_HEADER_LEN + 1] = {0};
  buf[0] = nodeAddress;
  buf[1] = gatewayAddress;
  buf[2] = msgIdACK;
  buf[3] = payloadLen;
  payload.getBytes(buf + LORA_HEADER_LEN, payloadLen+1);
  byte checksum = getChecksum(buf, payloadLen + LORA_HEADER_LEN + 1);
  // Serial.print("Checksum ACK: ");
  // Serial.println(checksum);
  LoRa.print(payload);         // add payload
  LoRa.write(checksum);        // add checksum
  LoRa.endPacket();            // finish packet and send it
}

// Function that parse a LoRa packet sent by a LoRa sensor node
// The format of the LoRa packet is:
// |  Receiver Addr  |  Sender Addr  |  Message ID  |  Payload Length  |  Payload data  |  Checksum  |
//      1 byte            1 byte          1 byte          1 byte            N byte          1 byte
void parseLoRaPacket(int packetSize) {
  // if there's no data, return
  if (packetSize == 0) return;

  // read packet header fields (4 bytes):
  byte receiverAddr = LoRa.read();  // receiver address
  byte senderAddr = LoRa.read();    // sender address
  byte msgID = LoRa.read();         // incoming message ID
  byte payloadLen = LoRa.read();    // incoming message length

  String data = "";

  // read payload of the packet
  for (uint16_t i = 0; LoRa.available() && i < payloadLen; i++) {
    data += (char)LoRa.read();
  }

  byte checksum = LoRa.read();

  byte buf[payloadLen + LORA_HEADER_LEN + 1] = {0};
  buf[0] = receiverAddr;
  buf[1] = senderAddr;
  buf[2] = msgID;
  buf[3] = payloadLen;
  data.getBytes(buf + LORA_HEADER_LEN, payloadLen+1);
  byte computedChecksum = getChecksum(buf, payloadLen + LORA_HEADER_LEN + 1);

  if (checksum != computedChecksum) {
    SerialPort.println("LoRa Error: Bad Checksum");
    SerialPort.print("Checksum received: ");
    SerialPort.println(checksum);
    SerialPort.print("Checksum computed: ");
    SerialPort.println(computedChecksum);
    SerialPort.println("Payload received:\n" + data);

    publishMsg = ("LoRa Error: Bad Checksum\nMsg: " + data);
    sendACK = true;
    return;
  }

  // if the receiver isn't this device discard the packet
  if (receiverAddr != gatewayAddress) {
    SerialPort.print("Received a packet with an unknow address: ");
    SerialPort.println(receiverAddr, HEX);
    SerialPort.println("Packet will be discarded.");
    // skip rest of function
    return;
  }

  // if message is for this device, print details about the received message:
  SerialPort.println("Packet Info:");
  SerialPort.println("Received from: 0x" + String(senderAddr, HEX));
  SerialPort.println("Sent to: 0x" + String(receiverAddr, HEX));
  SerialPort.println("Message ID: " + String(msgID));
  SerialPort.println("Message length: " + String(payloadLen));
  int rssi = LoRa.packetRssi();
  SerialPort.println("RSSI: " + String(rssi));
  float snr = LoRa.packetSnr();
  SerialPort.println("SNR: " + String(snr));
  SerialPort.println("Message payload:");
  SerialPort.println(data);
  SerialPort.println();

  publishMsg = ("ID: " + String(msgID) + "; Node: " + String(senderAddr));

  // Check if the received LoRa packet contains an alert message
  if (data.startsWith("#ALERT!")) {
    publishMsg += "; Type: EAM; ";
  }
  // message containing environmental data
  else if (data.startsWith("#TEMP=") && (data.indexOf("#HUM=") != -1) && (data.indexOf("#PRESS=") != -1)) {
    publishMsg += "; Type: EDP; ";
  }

  publishMsg += ("Length: " + String(payloadLen) + "; RSSI: " + String(rssi) + "; SNR: " + String(snr));
  
  if (lastMsgID == 0) {
    lastMsgID = msgID;
  }
  
  if (lastMsgID == msgID) {
    counterMsg++;
  }
  else {
    lastMsgID = msgID;
    counterMsg = 1;
  }

  msgIdACK = msgID;
  nodeAddress = senderAddr;
  sendACK = true;

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
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      Serial.println("Failed connecting to Adafruit IO through MQTT");
      // while (true);
      return;
    }
  }
  SerialPort.println("Connected!");
}

void loraSettingsCallback(char *data, uint16_t len) {
  String dataReceived = String(data);
  Serial.println("Data received: " + dataReceived);

  int startIndex, stopIndex;
    
  if (dataReceived.startsWith("BW")) {
    startIndex = dataReceived.indexOf("BW=") + String("BW=").length();
    stopIndex = dataReceived.indexOf(";SF=");
    long BW = dataReceived.substring(startIndex, stopIndex).toInt();
    Serial.print("BW = "); Serial.println(BW, DEC);
    BW = BW * 1E3;
    Serial.print("BW (Hz)= "); Serial.println(BW, DEC);
    
    startIndex = stopIndex + String(";SF=").length();
    stopIndex = dataReceived.indexOf(";CR=");
    int SF = dataReceived.substring(startIndex, stopIndex).toInt();
    Serial.print("SF = "); Serial.println(SF, DEC);

    startIndex = stopIndex + String(";CR=").length();
    int CR = dataReceived.substring(startIndex).toInt();
    Serial.print("CR = "); Serial.println(CR, DEC);

    LoRa.sleep();
    
    LoRa.setSignalBandwidth(BW);

    LoRa.setSpreadingFactor(SF);

    LoRa.setCodingRate4(CR);

    String msgACK = "LoRa settings updated: " + dataReceived + "\n";
    packetInfoFeed.publish(msgACK.c_str());

    delay(500);

    LoRa.receive();
  }
}

void printEncryptionType(int thisType) {
  // read the encryption type and print out the name:
  switch (thisType) {
    case ES_WIFI_SEC_OPEN:
      Serial.println("OPEN");
      break;
    case ES_WIFI_SEC_WEP:
      Serial.println("WEP");
      break;
    case ES_WIFI_SEC_WPA:
      Serial.println("WPA");
      break;
    case ES_WIFI_SEC_WPA2:
      Serial.println("WPA2");
      break;
    case ES_WIFI_SEC_WPA_WPA2:
      Serial.println("WPA_WPA2");
      break;
     case ES_WIFI_SEC_WPA2_TKIP:
      Serial.println("WPA_TKIP");
      break;
     case ES_WIFI_SEC_UNKNOWN:
      Serial.println("UNKNOW");
      break;
  }
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
    if (bssid[i] < 0x10) {
      Serial.print("0");
    }
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
    if (macAddr[i] < 0x10) {
      Serial.print("0");
    } 
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
  Serial.print("Subnet Mask: ");
  Serial.println(subMask);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");

  // print encryption type:
  Serial.print("Encryption type: ");
  printEncryptionType(WiFi.encryptionType());
}
