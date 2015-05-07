# si7005-in-RaspberryPI

## What's si7005?
si7005 is a tidy digital temperature/humidity sensor, using I2c protocal to communicate with MCU. you can get more information from http://www.silabs.com/products/sensors/humidity-sensors/Pages/Si7005.aspx


## What's this code used for?
When I used [RaspberryPi 2 mode B](https://www.raspberrypi.org/products/raspberry-pi-2-model-b/) to build a smart HomeCenter project, I found it is hard to use i2ctools or official driver to read si7005's data, and I searched the website, nobody do this, so I write this python code. 

according to official site：
>The Raspberry Pi 2 has an identical form factor to the previous (Pi 1) Model B+ and has complete compatibility with Raspberry Pi 1.

I think the code also can be used in (Pi 1) Model B+, but I'm not test it in that because I don't have PRi 1B+.

## How to use it?
Just copy the si7005.py to you project directory, and use it directly.

## Liscense
<wenhuix@qq.com> and <jack.xiao@outlook.com> wrote this file.  As long as you retain this notice you can do whatever you want with this stuff. If we meet some day, and you think this stuff is worth it, you can buy us a beer in return.   - Wenhui Xiang & Jack Xiao

Wenhui first wrote the original version and make it work smoothly on RPi. Then Jack refactor it using class to make it convenient to be called.
