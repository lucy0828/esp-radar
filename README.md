# Radar Human Presence Detection on Esp32

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
* Download ESPTOOL for flashing
* `pip install esptool`
* Move to the radar example folder and flash the bin file
* `esptool.py erase_flash`
* `esptool.py --baud 921600 write_flash 0x0000 ./build/esp-example-radar-presence.bin`
* Monitor the output
* `screen [SerialPort] 115200`


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

