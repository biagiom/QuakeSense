# QuakeSense

The QuakeSense project is an open-source earthquake and environmental monitoring system consisting of a low power and low cost IoT network made of energy-autonomous sensor nodes that are powered through an energy harvesting system and are connected to a gateway in a star topology.  
The project is based on two emerging IoT technologies, MQTT and LoRa, one of the most promising Low Power Wide Area Networks (LPWAN) technologies that provides a good compromise between coverage, current consumption, payload length, bandwidth, and data rate.  
The collected data are offered to users thanks to a dedicated web-based interface, thus allowing real-time monitoring of both seismic events and environmental parameters.

## Project implmentation

The main components of the QuakeSense project are: one or more sensor nodes, a single-channel LoRa gateway and an IoT cloud platform.

### Component description

#### Sensor Nodes

Each sensor node consists of the following components:
* [STM32 Nucleo F401RE](http://www.st.com/resource/en/user_manual/dm00105823.pdf) development board based on the STM32F401RE
  84 MHz ARM Cortex-M4 MCU with a Floating Point Unit (FPU), 512 kB of Flash memory and 96 kB of SRAM
    * Datasheet of STM32F401RE MCU: [http://www.st.com/resource/en/datasheet/stm32f401re.pdf](http://www.st.com/resource/en/datasheet/stm32f401re.pdf))
* [X-NUCLEO-IKS01A2](http://www.st.com/resource/en/data_brief/x-nucleo-iks01a2.pdf) motion MEMS and environmental sensor expansion board including:
  * LSM6DSL: MEMS 3D accelerometer and 3D gyroscope
    * Datasheet: [http://www.st.com/resource/en/datasheet/lsm6dsl.pdf](http://www.st.com/resource/en/datasheet/lsm6dsl.pdf)
  * LSM303AGR: MEMS 3D accelerometer and magnetometer
    * Datasheet: [http://www.st.com/resource/en/datasheet/lsm303agr.pdf](http://www.st.com/resource/en/datasheet/lsm303agr.pdf)
  * LPS22HB: MEMS pressure sensor
    * Datasheet: [http://www.st.com/resource/en/datasheet/lps22hb.pdf](http://www.st.com/resource/en/datasheet/lps22hb.pdf)
  * HTS221: capacitive digital relative humidity and temperature sensor
    * Datasheet: [http://www.st.com/resource/en/datasheet/hts221.pdf](http://www.st.com/resource/en/datasheet/hts221.pdf)
* [Dragino LoRa/GPS Shield](http://wiki.dragino.com/index.php?title=Lora/GPS_Shield) including:
  * RFM95W 137 MHz to 1020 MHz low-power, long-range LoRa RF transceiver
    * Datasheet: [http://www.hoperf.com/upload/rf/RFM95_96_97_98W.pdf](http://www.hoperf.com/upload/rf/RFM95_96_97_98W.pdf)
  * Quectel L80 GPS module based on the Mediatek MTK MT3339 All-in-One GPS system on a chip (SoC)
    * Datasheet: [https://www.quectel.com/UploadImage/Downlad/L80_Hardware_Design_V1.1.pdf](https://www.quectel.com/UploadImage/Downlad/L80_Hardware_Design_V1.1.pdf)
* [Seed Studio Solar Charger Shield v2.2](http://wiki.seeed.cc/Solar_Charger_Shield_V2.2) to which are connected:
  * the Adafruit 2000 mAh LiPo battery
    * Datasheet: [https://cdn-shop.adafruit.com/datasheets/LiIon2000mAh37V.pdf](https://cdn-shop.adafruit.com/datasheets/LiIon2000mAh37V.pdf)
  * the Seeed Studio 1.5 W solar panel
    * Datasheet: [http://wiki.seeedstudio.com/1.5W_Solar_Panel_81x137/](http://wiki.seeedstudio.com/1.5W_Solar_Panel_81x137/)

By default, each sensor node runs in Low-Power mode. In this configuration, the STM32 MCU runs in Stop mode (the MCU is stopped and the clocks are switched off, but SRAM and registers content are kept), the GPS module is in AlwaysLocate mode (an intelligent power saving mode that allows the GPS module to adaptively and automatically adjust the full on time according to the environmental and motion conditions), while the LoRa module in Standby mode for the maximum energy saving.  
When a seismic event occurs, the accelerometer generates an interrupt associated to the wake-up event and the node starts running in Run mode, which results in waking up the STM32 MCU and the GPS module in Full On mode, while the LoRa module is put in Transmit mode. The wake-up event happens if at least one of the 3 acceleration components exceeds the reference threshold (50 mg for the horizontal components, 1120 mg for the vertical one).  
Once in Run mode, the sensor node starts reading and recording acceleration data to compute the bracketed duration, defined as the time interval between the first and last exceeding of the acceleration threshold, and the 3 components of the Peak Ground Acceleration (PGA), defined as the maximum amplitude of acceleration in absolute value. The computed strong-motion parameters are also geo-referenced thanks to the GPS module, so that latitude, longitude, altitude, date and time paramters are added to the LoRa packet that is finally sent to the gateway.  
The baseline behavior of the node foresees a periodical (by default, every 15 minutes) monitoring of the environmental parameters (temperature, relative humidity and pressure) through the LPS22HB and HTS221 MEMS sensors.

#### Gateway

The single-channel LoRa gateway consists of the following components:

* [B-L475E-IOT01A2 STM32L4 Discovery kit](http://www.st.com/resource/en/user_manual/dm00347848.pdf) featuring:
  * STM32L475VG: Ultra-low-power ARM Cortex-M4 MCU which includes a Floating Point Unit (FPU), 1 MB of Flash memory and 128 kB of SRAM
    * Datasheet: [http://www.st.com/resource/en/datasheet/stm32l475vg.pdf](http://www.st.com/resource/en/datasheet/stm32l475vg.pdf)
  * Inventek ISM43362-M3G-L44: Wi-Fi module (802.11 b/g/n)
    * Datasheet: [http://www.inventeksys.com/wp-content/uploads/ISM43362_M3G_L44_Functional_Spec](http://www.inventeksys.com/wp-content/uploads/ISM43362_M3G_L44_Functional_Spec)
  * SPSGRF-868: Sub-GHz (868 Mhz) low-power RF module
    * Datasheet: [http://www.st.com/resource/en/datasheet/spsgrf.pdf](http://www.st.com/resource/en/datasheet/spsgrf.pdf)
  * SPBTLE-RF: Bluetooth v4.1 module
    * Datasheet: [http://www.st.com/resource/en/datasheet/spbtle-rf.pdf](http://www.st.com/resource/en/datasheet/spbtle-rf.pdf)
  * M24SR64-Y: dynamic NFC tag including also a printed NFC antenna
    * Datasheet: [http://www.st.com/resource/en/datasheet/m24sr64-y.pdf](http://www.st.com/resource/en/datasheet/m24sr64-y.pdf)
  * HTS221: capacitive relative humidity and temperature sensor
    * Datasheet: [http://www.st.com/resource/en/datasheet/hts221.pdf](http://www.st.com/resource/en/datasheet/hts221.pdf)
  * LSM303AGR: MEMS 3D accelerometer and MEMS 3D magnetometer
    * Datasheet: [http://www.st.com/resource/en/datasheet/lsm303agr.pdf](http://www.st.com/resource/en/datasheet/lsm303agr.pdf)
  * LSM6DSL: MEMS 3D accelerometer and MEMS 3D gyroscope
    * Datasheet: [http://www.st.com/resource/en/datasheet/lsm6dsl.pdf](http://www.st.com/resource/en/datasheet/lsm6dsl.pdf)
  * LSP22HB: 260-1260 hPa absolute digital output barometer
    * Datasheet: [http://www.st.com/resource/en/datasheet/lps22hb.pdf](http://www.st.com/resource/en/datasheet/lps22hb.pdf)
* Dragino LoRa Shield which includes:
  * a RFM95W low-power, long-range LoRa RF transceiver based on SX1276
    * Datasheet: [http://www.hoperf.com/upload/rf/RFM95_96_97_98W.pdf](http://www.hoperf.com/upload/rf/RFM95_96_97_98W.pdf)

The gateway is functionally in charge of receiving packets sent by sensor nodes, parsing the encapsulated values and
forwarding them to the Adafruit IO platform via the MQTT protocol.  
The gateway also deals with packets integrity: every time a new packet is received, the gateway computes the checksum and compares it with the one within the received message. In case of a mismatch, the packet is dropped and an error message is sent to the Adafruit IO platform.

#### Adafruit IO

The Adafruit IO platform is used to collect, process and visualize in real-time environmental data and strong-motion parameters related to seismic events.  
The user interface (UI) consists of a dashboard, which includes some widgets implemented through line graphs, gauges and other blocks to show the value of environmental parameters and the time trend of the three components of peak ground acceleration.

### Software components

The software libraries and frameworks used to implement the QuakeSense project are:

1. Arduino Core STM32: [https://github.com/stm32duino/Arduino_Core_STM32](https://github.com/stm32duino/Arduino_Core_STM32)
2. STM32LowPower library: [https://github.com/stm32duino/STM32LowPower](https://github.com/stm32duino/STM32LowPower)
3. Adafruit GPS library: [https://github.com/biagiom/Adafruit_GPS](https://github.com/biagiom/Adafruit_GPS)
4. Arduino LoRa library: [https://github.com/sandeepmistry/arduino-LoRa](https://github.com/sandeepmistry/arduino-LoRa)
5. WiFi-ISM43362-M3G-L44 library: [https://github.com/stm32duino/WiFi-ISM43362-M3G-L44](https://github.com/stm32duino/WiFi-ISM43362-M3G-L44)
6. LSM6DSL library: [https://github.com/stm32duino/LSM6DSL](https://github.com/stm32duino/LSM6DSL)
7. HTS221 library: [https://github.com/stm32duino/HTS221](https://github.com/stm32duino/HTS221)
8. LPS22HB library: [https://github.com/stm32duino/LPS22HB](https://github.com/stm32duino/LPS22HB)

## Authors
Biagio Montaruli - <b.montaruli@studenti.poliba.it>

## License
This software is licensed under the terms of the GNU GPLv3.
See the LICENSE.md file for more details.

## Acknowledgments
This project has been developed for my undergraduate thesis in Internet of Things at [Polytechnic University of Bari (PoliBa)](http://www.poliba.it/).  
I wish to say a special thanks to my professor and supervisor Luigi Alfredo Grieco, and to all researchers and people of the [Telematics' Lab](https://telematics.poliba.it/index.php?lang=en) (@telematics-dev) who helped me during the development of this project.
