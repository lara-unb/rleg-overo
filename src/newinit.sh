#!/bin/sh

# Setup MUX
devmem2 0x48002A18 w 0x00040100 # GPIO10 used as \SHDW
devmem2 0x480020D4 w 0x00040004 # GPIO66 and GPIO67

# Enable GPIOs

echo 168 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio168/direction

echo 64 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio64/direction

echo 176 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio176/direction

echo 65 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio65/direction

echo 66 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio66/direction

echo 67 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio67/direction
echo 0 > /sys/class/gpio/gpio67/value # For now, Everytime I try to change DAC output, it'll work

# Insert SPI protocol driver

