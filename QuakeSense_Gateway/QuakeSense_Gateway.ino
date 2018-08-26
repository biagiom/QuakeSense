/*
 *  QuakeSense_Gateway
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
 *       - Wi-Fi module Inventek ISM43362-M3G-L44 (802.11 b/g/n)
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
 *  to notify it about the sending of the data to Adafruit IoT Platform.
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
#include <WiFiUdpST.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <LoRa.h>
#include <time.h>

SPIClass SPI_3(PC12, PC11, PC10);
WiFiClass WiFi(&SPI_3, PE0, PE1, PE8, PB13);

/*************************************** WiFi Configuration *********************************************/

// Update these values according to your WiFi network
#define WLAN_SSID       "your_wifi_ssid"
#define WLAN_PASS       "your_wifi_password"

/*************************************** Adafruit.IO Setup **********************************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883  // use 8883 and WiFiClientSecure (if supported) for SSL/TLS
// Update the following values according to your Adafruit IO account
#define AIO_USERNAME    "your_aio_username"
#define AIO_KEY         "your_aio_key"

/********************************************** MQTT ****************************************************/

// Create a new object of WiFiClient class to connect to the MQTT server.
WiFiClient wifiClient;

// Redefine the connection keepalive time interval
#define MQTT_CONN_KEEPALIVE 1200

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&wifiClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/******************************************* Feeds ******************************************************/

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

bool isMqttConnected = false;

/**************************************** LoRa Config ***************************************************/
const int csPin = 10;         // LoRa radio chip select
const int resetPin = 5;       // LoRa radio reset
const int irqPin = 3;         // change for your board; must be a hardware interrupt pin

byte msgCount = 0;            // count of received messages
byte gatewayAddress = 0xBB;   // address of this device (LoRa Gateway)
byte nodeAddress;             // address of LoRa node to send ACK

bool sendACK = false;
byte msgIdACK;

// Set default LoRa mode (3):
// BW = 125 kHz; CR = 4/5; SF = 10
#define LORA_BW 125E3
#define LORA_SF 10
#define LORA_CR 5

#define LORA_HEADER_LEN 4

typedef enum {
  EAM_TYPE = 1,
  PGHAX = 2,
  PGHAY = 3,
  PGVA  = 4,
  DURATION = 5,
  LATITUDE = 6,
  LONGITUDE = 7,
  ALTITUDE = 8,
  DATE = 9,
  TIME = 10
} LORA_EAM_PACKET_FIELDS;

typedef enum {
  EDP_TYPE = 1,
  TEMP = 2,
  HUM  = 3,
  PRESS = 4
} LORA_EDP_PACKET_FIELDS;

/******************************************* Serial Config **********************************************/

#define SerialPort Serial
#define SERIAL_DEBUG 1

/****************************************** Network *****************************************************/

// NTP time stamp is in the first 48 bytes of the message
const int NTP_PACKET_SIZE = 48;
//buffer to hold incoming and outgoing packets
byte packetBuffer[NTP_PACKET_SIZE];
// IP Address of the NTP server: 188.213.165.209 => it.pool.ntp.org
IPAddress timeServer(188, 213, 165, 209);
unsigned long epoch = 0;
// Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
const unsigned long seventyYears = 2208988800UL;

String NTPTimestamp;

// Create an UDP instance to send and receive packets over UDP
WiFiUDP Udp;

typedef enum {
  UDP_STATUS_OK = 0,
  UDP_CONNECTION_ERROR = 1,
  UDP_WRITE_ERROR = 2,
  UDP_READ_ERROR = 3
} UDP_STATUS;

/******************************************* Sketch Code ************************************************/
void setup() {
  // Initialize serial communication at 115200 bps:
  SerialPort.begin(115200);
  // wait for the Serial port to open
  while (!SerialPort);
  delay(10);

  SerialPort.println("B-L475E-IOT01A Discovery Kit - LoRa Gateway");

  // Set CS, reset, IRQ pin for SX1276 LoRa module
  LoRa.setPins(csPin, resetPin, irqPin);

  // Set LoRa frequency band to 868 MHz
  if (!LoRa.begin(868E6)) {
    SerialPort.println("LoRa init failed. Check your connections.");
  }
  SerialPort.println("LoRa module successfully initialized.");

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);

  // register the receive callback
  LoRa.onReceive(parseLoRaPacket);

  // put the radio into receive mode
  LoRa.receive();

  // Initialize the WiFi module
  if (WiFi.status() == WL_NO_SHIELD) {
    SerialPort.println("WiFi Module ISM43362-M3G-L44 not detected");
    while (true);
  }

  // Print firmware version
  String fv = WiFi.firmwareVersion();
  SerialPort.print("Firmware version: ");
  SerialPort.println(fv);

  // Attempt to connect to Wifi network
  while (WiFi.status() != WL_CONNECTED) {
    SerialPort.print("Attempting to connect to the WiFi network ");
    SerialPort.println(WLAN_SSID);

    // Connect to WPA/WPA2 network.
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    // wait 8 seconds for connection:
    delay(8000);
  }
  SerialPort.println("WiFi connected:");
  printWifiStatus();
}

void loop() {
  // Ensure the connection to the MQTT server is alive:
  // this will make the first connection and automatically reconnect when disconnected.
  // See the MQTT_connect function definition further below.
  isMqttConnected = MQTT_connect();

  if (sendACK == true) {
    // Send ACK to LoRa Node:
    // alert message, earthquake and environmental data have been sent to Adafruit IO
    sendLoraACKPacket();
    sendACK = false;
    SerialPort.println("ACK sent to LoRa Node 0x" + String(nodeAddress, HEX));
    SerialPort.println();
    // wait 250 millis and go back into receive mode
    delay(250);
    LoRa.receive();
  }

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  // if (!mqtt.ping()) {
  //   mqtt.disconnect();
  // }
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
  payload.getBytes(buf + LORA_HEADER_LEN, payloadLen + 1);
  byte checksum = getChecksum(buf, payloadLen + LORA_HEADER_LEN + 1);
#if SERIAL_DEBUG == 1
  SerialPort.print("Checksum ACK: ");
  SerialPort.println(checksum);
#endif
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
  data.getBytes(buf + LORA_HEADER_LEN, payloadLen + 1);
  byte computedChecksum = getChecksum(buf, payloadLen + LORA_HEADER_LEN + 1);

  if (checksum != computedChecksum) {
    String errorMsg = "LoRa Error: Bad Checksum";
    SerialPort.println(errorMsg);
    SerialPort.print("Checksum received: ");
    SerialPort.println(checksum);
    SerialPort.print("Checksum computed: ");
    SerialPort.println(computedChecksum);
    SerialPort.println("Payload received:\n" + data);

    // Send an error message to the Adafruit IO platform
    alertFeed.publish(errorMsg.c_str());
    return;
  }

  // if the receiver isn't this device discard the packet
  if (receiverAddr != gatewayAddress) {
    SerialPort.print("Received a new packet with an unknow address: ");
    SerialPort.println(receiverAddr, HEX);
    SerialPort.println("Packet will be discarded.");
    // skip rest of function
    return;
  }

  // if message is for this device, print details about the received message:
  SerialPort.println("New LoRa packet received:");
  SerialPort.println("Received from: 0x" + String(senderAddr, HEX));
  SerialPort.println("Sent to: 0x" + String(receiverAddr, HEX));
  SerialPort.println("Message ID: " + String(msgID));
  SerialPort.println("Message length: " + String(payloadLen));
  int16_t rssi = LoRa.packetRssi();
  SerialPort.println("RSSI: " + String(rssi));
  float snr = LoRa.packetSnr();
  SerialPort.println("SNR: " + String(snr));
  SerialPort.println("Message payload:");
  SerialPort.println(data);
  SerialPort.println();


  int8_t startIndex, stopIndex;
  String fieldValue = "";
  String alertMsg;

  // A LoRa Node can send two types of messages: EAM (Eartquake Alert Message) and EDP (Environmental Data Packet)
  //
  // Payload format of an EAM LoRa packet:
  // 1) EAMF: FULL FORMAT (max 72 byte)
  //    |  Type  |  PGHAX  |  PGHAY  |  PGVA  |  Bracketed duration  |  Latitude  |  Longitude |  Altitude  |  Date (DD/MM/YYYY)  |  Time (HH:MM:SS)  |
  //      5 byte    5 byte   5 byte    5 byte          6 byte            9 byte       9 byte       8 byte              11 byte            9 byte
  // 2) EAMR: REDUCED FORMAT FORMAT (LoRaWAN-compliant: max 44 byte)
  //    |  Type  |  PGHAX  |  PGHAY  |  PGVA  |  Bracketed duration  |  Latitude  |  Longitude |
  //      5 byte    5 byte   5 byte    5 byte          6 byte            9 byte       9 byte
  // 3) EAMB: BASE FORMAT (max 26 byte)
  //    |  Type  |  PGHAX  |  PGHAY  |  PGVA  |  Bracketed duration  |
  //      5 byte    5 byte   5 byte    5 byte          6 byte
  //
  // Payload format of an EDP LoRa packet (max 27 byte):
  // |  Type  |  TEMPERATURE  |  HUMIDITY  |  PRESSURE  |
  //   5 byte      7 byte         7 byte       8 byte
  //
  if ((data.startsWith("#EAM") || data.startsWith("#EDP")) && isMqttConnected == true) {
    uint8_t field;
    uint8_t numMaxFields = 10;
    startIndex = 1;
    stopIndex = 5;

    String packetType = data.substring(startIndex, stopIndex);
    for (field = 2; field <= numMaxFields; field++) {
      startIndex = data.indexOf("#", stopIndex) + 1;
      stopIndex = data.indexOf("#", startIndex);

      if (startIndex != -1 && stopIndex != -1 && stopIndex > startIndex) {
        fieldValue = data.substring(startIndex, stopIndex);

        if (packetType.startsWith("EAM")) {
          switch (field) {
            case PGHAX:
              pghaxFeed.publish(fieldValue.toInt());
              alertMsg = ("EAM - PGHAX: " + fieldValue + ", ");
            #if SERIAL_DEBUG == 1
              SerialPort.print("PGHAX: "); SerialPort.println(fieldValue);
            #endif
              break;
            case PGHAY:
              pghayFeed.publish(fieldValue.toInt());
              alertMsg += ("PGHAY: " + fieldValue + ", ");
            #if SERIAL_DEBUG == 1
              SerialPort.print("PGHAY: "); SerialPort.println(fieldValue);
            #endif
              break;
            case PGVA:
              pgvaFeed.publish(fieldValue.toInt());
              alertMsg += ("PGVA: " + fieldValue + ", ");
            #if SERIAL_DEBUG == 1
              SerialPort.print("PGVA: "); SerialPort.println(fieldValue);
            #endif
              break;
            case DURATION:
              alertMsg += ("Duration: " + fieldValue + "\n");
            #if SERIAL_DEBUG == 1
              SerialPort.print("Duration: "); SerialPort.println(fieldValue);
            #endif
              break;
            case LATITUDE:
              if (packetType.equals("EAMB"))
                break;
              alertMsg += ("LAT: " + fieldValue + ", ");
            #if SERIAL_DEBUG == 1
              SerialPort.print("Latitude: "); SerialPort.println(fieldValue);
            #endif
              break;
            case LONGITUDE:
              if (packetType.equals("EAMB"))
                break;
              alertMsg += ("LON: " + fieldValue + ", ");
            #if SERIAL_DEBUG == 1
              SerialPort.print("Longitude: "); SerialPort.println(fieldValue);
            #endif
              break;
            case ALTITUDE:
              if (packetType.equals("EAMR") || packetType.equals("EAMB"))
                break;
              alertMsg += ("ALT: " + fieldValue + "\n");
            #if SERIAL_DEBUG == 1
              SerialPort.print("Altitude: "); SerialPort.println(fieldValue);
            #endif
              break;
            case DATE:
              if (packetType.equals("EAMR") || packetType.equals("EAMB"))
                break;
              alertMsg += ("DATE: " + fieldValue + ", ");
            #if SERIAL_DEBUG == 1
              SerialPort.print("Date: "); SerialPort.println(fieldValue);
            #endif
              break;
            case TIME:
              if (packetType.equals("EAMR") || packetType.equals("EAMB"))
                break;
              alertMsg += ("TIME: " + fieldValue + "\n\n");
            #if SERIAL_DEBUG == 1
              SerialPort.print("Time: "); SerialPort.println(fieldValue);
            #endif
              break;
            default:
              SerialPort.print("Unknown field: ");
              SerialPort.println(fieldValue);
              break;
          }
        }
        else if (packetType.equals("EDPB")) {
          switch (field) {
            case TEMP:
              tempFeed.publish(fieldValue.toFloat());
            #if SERIAL_DEBUG == 1
              SerialPort.print("Temperature: "); SerialPort.println(fieldValue);
            #endif
              break;
            case HUM:
              humFeed.publish(fieldValue.toFloat());
            #if SERIAL_DEBUG == 1
              SerialPort.print("Humidity: "); SerialPort.println(fieldValue);
            #endif
              break;
            case PRESS:
              pressFeed.publish(fieldValue.toFloat());
            #if SERIAL_DEBUG == 1
              SerialPort.print("Pressure: "); SerialPort.println(fieldValue);
            #endif
              break;
            default:
              SerialPort.print("Unknown field: ");
              SerialPort.println(fieldValue);
              break;
          }
        }
        else {
        #if SERIAL_DEBUG == 1
          SerialPort.print("Invalid packet type: ");
          SerialPort.println(packetType);
        #endif
        }
      }
      else {
        break;
      }
    }

    if (packetType.equals("EAMR") || packetType.equals("EAMB")) {
      if (updateNTP() == UDP_STATUS_OK) {
        alertMsg += ("TIMESTAMP: " + NTPTimestamp + "\n\n");
      }
    }

    if (packetType.startsWith("EAM")) {
    #if SERIAL_DEBUG == 1
      SerialPort.println("Alert Message:");
      SerialPort.print(alertMsg);
    #endif
      alertFeed.publish(alertMsg.c_str());
    }

    msgIdACK = msgID;
    nodeAddress = senderAddr;
    sendACK = true;
  }
  else {
    String errorMsg = "LoRa Error: Invalid type of packet received";
    SerialPort.println(errorMsg);
    alertFeed.publish(errorMsg.c_str());
  }

  SerialPort.println("------------------------------------------");
}

// Function to connect and reconnect as necessary to the MQTT server (Adafruit IO MQTT Broker).
// Should be called in the loop function and it will take care if connecting.
bool MQTT_connect(void) {
  int8_t mqttStatus;
  bool mqttFailedConnecting = false;

  // Stop if already connected.
  if (mqtt.connected()) {
    return true;
  }

  SerialPort.println("Connecting to Adafruit IO through MQTT...");

  uint8_t retries = 3;
  // connect will return 0 for connected
  while ((mqttStatus = mqtt.connect()) != 0) {
    SerialPort.println(mqtt.connectErrorString(mqttStatus));
    SerialPort.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      SerialPort.println("Failed connecting to Adafruit IO through MQTT");
      mqttFailedConnecting = true;
      break;
    }
  }

  if (mqttFailedConnecting == false) {
    SerialPort.println("Connected!");
    return true;
  }
  else {
    return false;
  }
}

void printEncryptionType(int thisType) {
  // read the encryption type and print out the name:
  switch (thisType) {
    case ES_WIFI_SEC_OPEN:
      SerialPort.println("OPEN");
      break;
    case ES_WIFI_SEC_WEP:
      SerialPort.println("WEP");
      break;
    case ES_WIFI_SEC_WPA:
      SerialPort.println("WPA");
      break;
    case ES_WIFI_SEC_WPA2:
      SerialPort.println("WPA2");
      break;
    case ES_WIFI_SEC_WPA_WPA2:
      SerialPort.println("WPA_WPA2");
      break;
    case ES_WIFI_SEC_WPA2_TKIP:
      SerialPort.println("WPA_TKIP");
      break;
    case ES_WIFI_SEC_UNKNOWN:
      SerialPort.println("UNKNOW");
      break;
  }
}

// Function that prints WiFi status and connection parameters.
void printWifiStatus() {
  // print the SSID of the network you're connected to:
  SerialPort.print("SSID: ");
  SerialPort.println(WiFi.SSID());

  // print the BSSID of the network you're connected to:
  byte bssid[6];
  WiFi.BSSID(bssid);

  SerialPort.print("BSSID: ");
  for (uint8_t i = 0; i < 6; i++) {
    if (bssid[i] < 0x10) {
      SerialPort.print("0");
    }
    SerialPort.print(bssid[i], HEX);
    if (i != 5) {
      SerialPort.print(":");
    }
    else {
      SerialPort.println();
    }
  }

  // print the MAC address of the WiFi module:
  byte macAddr[6];
  WiFi.macAddress(macAddr);

  SerialPort.print("MAC Address: ");
  for (uint8_t i = 0; i < 6; i++) {
    if (macAddr[i] < 0x10) {
      SerialPort.print("0");
    }
    SerialPort.print(macAddr[i], HEX);
    if (i != 5) {
      SerialPort.print(":");
    }
    else {
      SerialPort.println();
    }
  }

  // print IP address of the WiFi module assigned through DHCP:
  IPAddress ip = WiFi.localIP();
  SerialPort.print("IP Address: ");
  SerialPort.println(ip);

  // print the subnet mask of the WiFi network you're connected to:
  IPAddress subMask = WiFi.subnetMask();
  SerialPort.print("Subnet Mask: ");
  SerialPort.println(subMask);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  SerialPort.print("Signal strength (RSSI): ");
  SerialPort.print(rssi);
  SerialPort.println(" dBm");

  // print encryption type:
  SerialPort.print("Encryption type: ");
  printEncryptionType(WiFi.encryptionType());
}

// send an NTP request to the time server at the given address
byte sendNTPpacket(IPAddress &serverIP) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  byte ret;

  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;            // Stratum, or type of clock
  packetBuffer[2] = 6;            // Polling Interval
  packetBuffer[3] = 0xEC;         // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // Send a packet requesting a timestamp.
  // NTP requests are to port 123
  if (Udp.beginPacket(serverIP, 123) == 0) {
    SerialPort.print("Failed to start UDP connection to NTP Server ");
    SerialPort.println(serverIP);
    return UDP_CONNECTION_ERROR;
  }

  size_t pktSentLen = Udp.write(packetBuffer, NTP_PACKET_SIZE);
  if (pktSentLen == 0) {
    SerialPort.println("Failed to send the UDP packet.");
    return UDP_WRITE_ERROR;
  }

#if SERIAL_DEBUG == 1
  SerialPort.print("NTP packet length: ");
  SerialPort.println(pktSentLen);
#endif

  return UDP_STATUS_OK;
}

String getTimestamp(unsigned long epoch, unsigned long timeOffset) {
  time_t rawtime = epoch + timeOffset;
  struct tm * ti;
  ti = localtime(&rawtime);

  uint16_t year = ti->tm_year + 1900;
  String yearStr = String(year);

  uint8_t month = ti->tm_mon + 1;
  String monthStr = month < 10 ? "0" + String(month) : String(month);

  uint8_t day = ti->tm_mday;
  String dayStr = day < 10 ? "0" + String(day) : String(day);

  uint8_t hours = ti->tm_hour;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  uint8_t minutes = ti->tm_min;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  uint8_t seconds = ti->tm_sec;
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);

  return (dayStr + "/" + monthStr + "/" + yearStr + " " +
          hoursStr + ":" + minuteStr + ":" + secondStr);
}

byte updateNTP(void) {
  // start a new NTP packet
  
  SerialPort.println("Sending a new NTP request...");
  byte connectionStatus = sendNTPpacket(timeServer);

  if (connectionStatus == UDP_STATUS_OK) {
    // wait to see if a reply is available
    delay(1000);

    // read the packet into the buffer
    int packetSize = Udp.read(packetBuffer, NTP_PACKET_SIZE);
    
    if (packetSize > 0) {
      SerialPort.println("New NTP packet received");

      // the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, esxtract the two words:
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      // now convert NTP time into everyday time subtracting seventy years:
      epoch = secsSince1900 - seventyYears;

    #if SERIAL_DEBUG == 1
      SerialPort.print("Seconds since Jan 1 1900: ");
      SerialPort.println(secsSince1900);
      // print Unix time:
      SerialPort.print("Unix time: ");
      SerialPort.println(epoch);
    #endif
      // print time and date
      NTPTimestamp = getTimestamp(epoch, 3600);
      SerialPort.print("NTP Timestamp: ");
      SerialPort.println(NTPTimestamp);

      Udp.stop();
      delay(500);
    }
    else {
      return UDP_READ_ERROR;
    }
  }
  
  return connectionStatus;
}
