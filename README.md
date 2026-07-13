# RNode Firmware for ESP32 + EBYTE E32-170T30D

E32-170T30D is 1W 170MHz module, have no SPI interface, UART only.  
Warning! This firmware is not compartible with others.  

## Boards currently supported:
ESP32C3  
ESP32 DOIT DEVKIT v1  

## ESP32 can be connected to RNodeInterface by USB Serial, WIFI or BLE.

Frequency can be 160-173.5MHZ with 250KHz step.  
Air data rate can be controlled by SF (7-12) parameter in interface.  

## For NVS partition create secrets.ini file with data like this:  
```
[esp32doit-devkit-v1]
NVS_WIFISSID = yourssid
NVS_WIFIPSK  = yourpasswd
NVS_WIFION   = 1
NVS_BLEON    = 0

[esp32c3_mini]
NVS_WIFISSID = no
NVS_WIFIPSK  = no
NVS_WIFION   = 0
NVS_BLEON    = 1
```

<img src="folder-name/1.jpg">
<img src="folder-name/2.jpg">