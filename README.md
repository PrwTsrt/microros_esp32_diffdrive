
# MicroROS-ESP32-Diffdrive

This is a ROS2 and micro-ROS project to control a differential drive robot using an [Yahboom microROS control board](http://www.yahboom.net/study/MicroROS-Board) (ESP32-S3 module). I'm using Ubuntu 22.04 and [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) running on an orange pi 5b. The connection to ESP32 that running micro-ros is via USB/UART. 



## Setting up ESP-IDF Development Environment 

The first few steps are taken directly from the [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/index.html).

### Install Prerequisites
Install related dependencies
```bash
  sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```
### Get ESP-IDF
Download the esp-idf-v5.1.2 version
```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.1.2 --recursive https://github.com/espressif/esp-idf.git
```
### Set up the Tools
Install the tools required by ESP-IDF, modifying 'esp32s3' to match the chip you are using.
```bash
cd ~/esp/esp-idf
./install.sh esp32s3
```
### Set up the Environment Variables
Add the installed tools to the PATH environment variable.To make the tools usable from the command line.
```bash
source ~/esp/esp-idf/export.sh
```
### Confige, compile and flash firmware
Now connect your ESP32 board to the computer and check under which serial port the board is visible. Navigate to your directory, set ESP32-S3 as the target, and run the project configuration utility menuconfig. You are using this menu to set up project variables.
```bash
cd ~/esp/esp-idf/examples/get-started/hello_world
idf.py set-target esp32s3
idf.py menuconfig
```
Build the project by running
```bash
idf.py build
```
If there are no errors, the build finishes by generating the firmware binary .bin files. Then, run the following command to flash the firmware onto the board.
```bash
idf.py flash
```
Launches the IDF Monitor application
```bash
idf.py monitor
```
Shortcut command
```bash
idf.py build flash monitor
```
