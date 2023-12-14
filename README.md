# XENSIV™ 60-GHz Radar on Esp32

## Overview

This code example demonstrates Infineon's radar presence solution to detect human presence within a configurable distance. Powered by the XENSIV™ 60-GHz radar, this solution provides extremely high accuracy in detecting both micro and macro motions. The ability to detect micro motion offers unique benefits over conventional technologies deployed to detect human presence, thus making it perfect for user interaction with devices.

This code example is ported from official [ModusToolbox™](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/) code example on [Github/Infineon](https://github.com/Infineon/mtb-example-psoc6-radar-presence). For *Operation* instructions, please refer to the original [README.md](https://github.com/Infineon/mtb-example-psoc6-radar-presence/blob/master/README.md) for more details. 

## Setup Environment
* `conda create --name env`
* `conda activate esp`
* `mkdir esp` `cd esp`
* [ESP-IDF Toolchain Setup](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html)
* `git clone https://github.com/espressif/esp-idf.git`
* `cd esp-idf`
* `./install.sh all`
* `. ./export.sh`
* `cd examples`
* Copy this radar folder inside example folder
* `idf.py build`
* Flash the files
* `idf.py flash`
* Monitor the output
* `idf.py monitor`
* To modify the WiFi, go to menuconfig and change the configuration
* `idf.py menuconfig`
* To change the radar settings, go to ./main/radar settings folder, find the target settings, copy and paste the target setting in the ./main/radar_settings_tr13.h

## Running server to collect radar data over UDP
* Go to scripts folder and run the udp server
* `python run_tcp_server.py --port=3333`
* Go to main folder and run `idf.py monitor` to monitor Esp32
* Your radar data will be saved as csv file inside the scripts folder

## Versioning 
This code example adopts the following versioning convention:  `a.b.c+esp32-#`, where 
- `a.b.c` refers to the upstream official version it's ported from 
- `esp32` refers to the MCU platform 
- `#` refers to the build ID 

## How-to
This is a complete code example with `sdkconfig` for esp32 MCU platform.

### Tested Environment
Tested in `esp-idf-v5.0.1` with VSCode on Windows using [ESP-IDF extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension) 

### Tested Devices 
Tested with the following hardware:
* Adafruit ESP32 Feather V2 (Product ID [5400](https://www.adafruit.com/product/5400))
* CSK Radar Wing Board with TR13C
* CSK Radar Adaptor Wing Board + TR13C Shiled 
* CSK Radar Adaptor Wing Board + UTR11 Shield 

