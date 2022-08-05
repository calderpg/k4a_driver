# k4a_driver
Basic ROS 1 and ROS 2 driver for K4A devices

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

3. Install ROS 1 Melodic+ _or_ ROS 2 Galactic+

4. Symlink the corresponding `CMakeLists.txt` and `package.xml` for the ROS
distribution you are using:

*For ROS 1 Melodic+*
```sh
cd ~/ws/src/k4a_driver
ln -sT CMakeLists.txt.ros1 CMakeLists.txt
ln -sT package.xml.ros1 package.xml
```

*For ROS 2 Galactic+*
```sh
cd ~/ws/src/k4a_driver
ln -sT CMakeLists.txt.ros2 CMakeLists.txt
ln -sT package.xml.ros2 package.xml
```

## Run

Run with

ROS 1: `~$ rosrun k4a_driver k4a_driver_node`

ROS 2: `~$ ros2 run k4a_driver k4a_driver_node`
