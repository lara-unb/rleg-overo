#!/bin/sh

# Setup MUX
devmem2 0x48002A18 w 0x00040100 # GPIO10 used as \SHDW

# Enable GPIOs
echo 10 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio10/direction

echo 186 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio186/direction

# Insert SPI protocol driver
