# Fork Description
Todo:
- bring communication with the controller and thermometers in accordance with the datasheet
- add support of software bitbang I2C library
- write init for thermometers
- optimization, etc.

The original library did not have the support of the multichannel of the DS-800 chip, so the work is focused on the support of a single-channel controller.

# Original Description
 Communication driver for DS2482-800 working with DS18B20 sensors
 
 8-Channel 1-Wire Master - https://datasheets.maximintegrated.com/en/ds/DS2482-800.pdf
 
 1-Wire Digital Thermometer - https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf

## Features:

 Parasitic Power Supply for all 1-wire devices online. 
 
 One Wire Search - find and identify (64bit ROM ID) all 1-wire devices online.
 
 Read Temperature - read temperature from individual or all 1-wire devices.
 
//Channel select feature to be added shortly.

## I2C
I2C procedures included in this driver are platform specific and you may have to make small changes to suit your own I2C hardware.
I2C communication examples are included in the chip's datasheet like below.

![image](https://user-images.githubusercontent.com/50047346/179456609-69d216ea-8ec7-4bd6-82d5-a512dbf72908.png)
