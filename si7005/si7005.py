#!/usr/bin/env python
# -*- coding:utf-8 -*-

'si7005 temperature and humidity device'

__author__ = 'wenhuix'

import RPi.GPIO as GPIO
import smbus
import time

addr = 0x40

#Si7005 Registers
REG_STATUS = 0x00
REG_DATA = 0x01
REG_CONFIG = 0x03
REG_ID = 0x11

#config register
CONFIG_START = 0x01
CONFIG_HEAT = 0x02
CONFIG_HUMIDITY = 0x00
CONFIG_TEMPERATURE = 0x10
CONFIG_FAST = 0x20

#ID register
ID_SI7005 = 0x50

#status register
STATUS_NOT_READY = 0x01

#coefficients
TEMPERATURE_OFFSET = 50
TEMPERATURE_SLOPE = 32
HUMIDITY_OFFSET = 24
HUMIDITY_SLOPE = 16
SCALAR = 16384
A0 = -78388
A1 = 6567
A2 = -64
Q0 = 3233
Q1 = 39

temperature = 25000
config_reg = 0

def _si7005_measure(addr, config):

    time.sleep(0.05)
    error = bus.write_byte_data(addr, REG_CONFIG, CONFIG_START|config|config_reg)
    if error:
        print 'error = ' + error
        return

    status = STATUS_NOT_READY
    while status & STATUS_NOT_READY:
        status = bus.read_byte_data(addr, REG_STATUS)
        if status < 0:
            print 'error, status = ' + status

    error = bus.write_byte_data(addr, REG_CONFIG, 0)
    if error:
        print 'error = ' + error
        return

    data = bus.read_word_data(addr, REG_DATA)
    if data < 0:
        value = data
    else:
        value = (data & 0x000000FF) * 256 + data & 0x0000FF00

    return value

# set gpio num, based on board mode
def set_gpio(num = 11):
    global gpio_num
    gpio_num = num
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(gpio_num, GPIO.OUT) 
    return

# set i2c bus num: 0 or 1
def set_i2cbus_num(num = 1):
    global bus_num
    bus_num = num
    global bus
    bus = smbus.SMBus(bus_num)
    return

# detect sensor
def detect_sensor():
    GPIO.output(gpio_num, GPIO.LOW)
    time.sleep(0.05)
    id = bus.read_byte_data(addr, REG_ID)
    GPIO.output(gpio_num, GPIO.HIGH)
    if id != ID_SI7005:
        return 0
    return 1

# heater
def enable_heater():
    global config_reg
    config_reg |= CONFIG_HEAT
    return

def disable_heater():
    global config_reg
    config_reg ^= CONFIG_HEAT
    return

# fast measure
def enable_fastmeasure():
    global config_reg
    config_reg |= CONFIG_FAST
    return

def disable_fastmeasure():
    global config_reg
    config_reg ^= CONFIG_FAST
    return

# temperature
def getTemperature():
    global temperature 

    GPIO.output(gpio_num, GPIO.LOW)
    value = _si7005_measure(addr, CONFIG_TEMPERATURE)
    temperature = (((value>>2)*1000)/TEMPERATURE_SLOPE) - (TEMPERATURE_OFFSET*1000)
    temperature /= 1000.0
    GPIO.output(gpio_num, GPIO.HIGH)

    return temperature

# humidity
def getHumidity():

    GPIO.output(gpio_num, GPIO.LOW)
    value = _si7005_measure(addr, CONFIG_HUMIDITY)
    curve = ((value>>4)*1000)/HUMIDITY_SLOPE - HUMIDITY_OFFSET*1000
    linear = (curve * SCALAR - (curve*curve*A2) / 1000 - curve*A1 - A0 * 1000) / SCALAR
    linear = (linear*SCALAR + (temperature - 30000) * ((linear*Q1) / 1000 + Q0)) / SCALAR
    if linear < 0:
        humidity = 0
    elif linear > 100000:
        humidity = 100000
    else:
        humidity = linear
    humidity /= 1000.0
    GPIO.output(gpio_num, GPIO.HIGH)

    return humidity


if __name__=='__main__':
    set_gpio()
    set_i2cbus_num()
    print detect_sensor()
    enable_fastmeasure()
    enable_heater()
    temp = getTemperature()
    print 'temperature = %3.3f' % temp
    humi = getHumidity()
    print 'humidity = %3.3f' % humi
