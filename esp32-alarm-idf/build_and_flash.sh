#!/bin/bash

# ESP-IDF Docker build and flash script for ESP32-C3

echo "Building ESP32-C3 Alarm System with ESP-IDF Docker..."

# Build the project
docker run --rm -v $PWD:/project -w /project espressif/idf:latest idf.py set-target esp32c3
docker run --rm -v $PWD:/project -w /project espressif/idf:latest idf.py build

echo "Build complete!"
echo ""
echo "To flash to your ESP32-C3:"
echo "1. Connect your ESP32-C3 to USB"
echo "2. Find the serial port (usually /dev/ttyUSB0 or /dev/ttyACM0 on Linux, COM3 etc on Windows)"
echo "3. Run:"
echo "   docker run --rm -v $PWD:/project -w /project --device=/dev/ttyUSB0 espressif/idf:latest idf.py -p /dev/ttyUSB0 flash"
echo ""
echo "To monitor serial output:"
echo "   docker run --rm -v $PWD:/project -w /project --device=/dev/ttyUSB0 espressif/idf:latest idf.py -p /dev/ttyUSB0 monitor"
echo ""
echo "To build, flash and monitor in one command:"
echo "   docker run --rm -v $PWD:/project -w /project --device=/dev/ttyUSB0 espressif/idf:latest idf.py -p /dev/ttyUSB0 flash monitor"