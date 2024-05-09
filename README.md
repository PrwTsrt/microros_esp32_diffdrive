# MicroROS-ESP32-Diffdrive

This is a ROS2 and micro-ROS project to control a differential drive robot using an [Yahboom microROS control board](http://www.yahboom.net/study/MicroROS-Board) (ESP32-S3 module). I'm using Ubuntu 22.04 and [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) running on an orange pi 5b. The connection to ESP32 that running micro-ros is via USB/UART. 

## Setting up ESP-IDF Development Environment 

The first few steps are taken directly from the [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/index.html).

### 1. Install Prerequisites

Open the terminal and run the following command.

```bash
  sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

### 2. Install Prerequisites

Open the terminal and run the following command.

```bash
  sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```
