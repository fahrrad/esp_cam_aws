# Requirements

This project needs to be build with ESP-IDF toolkit, V5.1. This version is important, as the release version has a annoying bug that will require you to change some code in a library that is included.

The easiest way to install the SDK is :

- Install VSCode
- Install the ESP extension, and configure the extension (seee the description [here](https://docs.espressif.com/projects/esp-idf/en/v4.2.3/esp32/get-started/vscode-setup.html))

# Building & Flashing

After the tool is installed, you should be able to build the project. First, set the target. If you are using an ESP32-CAM, the CLI command is `idf.py set-target esp32 `. If you are in VSCode, you can also execute `ESP-IDF: set target device`. The shortcut to type in a command in VSCode is `CMD-Shift-P` on mac. On windows, it's `F1`. While you are at it, configure the serial port to use (this requires your device to be connected to the computer). This is only possible in VSCOde (`ESP-IDF: select port to use`). For the CLI you will need to add `--port ...` to the other commands.

The next step is to configure the build with your WiFi credentials, and AWS-IoT endpoint. The easiest way is to do `idf.py menuconfig` in the CLI, of `ESP-IDF: menuconfig` in VSCode. You will need to set 3 values:

- WIFI SSID
- WIFI password
- AWS IoT endpoint.

After that, you should be all set. With the device plugged is, use `idf.py flash monitor`
. This command will build and flash the device. When that is done, it should restart the device and start monitoring automatically.
