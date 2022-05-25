# k4a_driver
Basic ROS driver for K4A devices

## Prerequisites

1. Install [K4A SDK](https://docs.microsoft.com/en-us/azure/Kinect-dk/sensor-sdk-download)

2. Add the following to `/etc/udev/rules.d/90-k4a.rules`

```
# This file belongs in /etc/udev/rules.d/90-k4a.rules
# ATTR{product}=="Kinect4Azure"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="097a", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="097b", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="097c", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="097d", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="097e", MODE="0666"
```

3. Install ROS 2

## Run

Run with `~$ ros2 run k4a_driver k4a_driver_node`
