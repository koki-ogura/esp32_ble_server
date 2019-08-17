# esp32_ble_server
## How to install
### clone this package
```.sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/koki-ogura/esp32_ble_server.git
```
### setting esp32 device with udev
#### insert esp32 device that already installed esp32_ble_cont firmware
##### check device name (/dev/ttyUSB0, /dev/ttyUSB1, ...)
```.sh
$ ls -al /dev/ttyUSB*
```
##### check usb info (ex. /dev/ttyUSB0)
```.sh
$ udevadm info -a -p $(udevadm -q path -n /dev/ttyUSB0)
```
search idProduct and idVendor, then look above KERNELS.
##### modify blecon.rules file (ex. KERNELS=="1-3.3")
```.sh
$ cd ~/catkin_ws/src/esp32_ble_server/udev
```
modify blecon.rules file's KERNELS part. and  
```.sh
$ sudo cp blecon.rules /etc/udev/rules.d
$ sudo service udev reload
$ sudo service udev restart
```
#### replug usb and check /dev/blecon
replug esp32 device  
```.sh
$ ls -al /dev/blecon
```
### make esp32_ble_server
```.sh
$ cd ~/catkin_ws/src
$ catkin_make
```

## run esp32_ble_server
terminal1  
```.sh
$ roscore
```

terminal2  
```.sh
$ rosrun esp32_ble_server esp32_ble_server_node.py
```

## use esp32_ble_server
terminal3  
```.sh
$ rostopic echo /ble_msg
```
run iPhone's BleROSCont app.  
and you can read connected message on terminal2.  
check push iPhone P1 or P2 button.  

terminal4  
```.sh
$ rosservice call /ble_srv "test message"
```
this message will appear on your iPhone.  
