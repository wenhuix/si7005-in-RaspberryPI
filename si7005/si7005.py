''' -WARE LICENSE" (Revision 42):
 * <wenhuix@foxmail.com> and <jack.xiao@outlook.com> wrote this file.  As long 
 * as you retain this notice you can do whatever you want with this stuff. If 
 * we meet some day, and you think this stuff is worth it, you can buy us a 
 * beer in return.   - Wenhui Xiang & Jack Xiao

 * Wenhui first wrote the original version and make it work smoothly on RPi.
 * Then Jack refactor it using class to make it convenient to be called.
 * ----------------------------------------------------------------------------
'''

'si7005 temperature and humidity device'

__author__ = 'wenhuix'

import RPi.GPIO as GPIO
import smbus
import time

SI7005_ADR = 0x40

# Si7005 Registers
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


class si7005:
    def __init__(self, cspin = 11):
        self._cs_pin = cspin
        self.config_reg = 0
        self.temperature = 25000

        self.set_gpio()
        self.set_i2cbus_num()
        self.enable_fastmeasure()
        self.enable_heater()
        self.detect_sensor()
        return

    # set gpio num, based on board mode
    def set_gpio(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._cs_pin, GPIO.OUT) 
        return

    # set i2c bus num: 0 or 1
    def set_i2cbus_num(self, bus_num = 1):
        self.bus = smbus.SMBus(bus_num)
        return

    # detect sensor
    def detect_sensor(self):
        GPIO.output(self._cs_pin, GPIO.LOW)
        time.sleep(0.05)
        id = self.bus.read_byte_data(SI7005_ADR, REG_ID)
        GPIO.output(self._cs_pin, GPIO.HIGH)
        if id != ID_SI7005:
            print "Warning: si7005 id is not correct: %d" %id
            return 0
        return 1

    # heater
    def enable_heater(self):
        self.config_reg |= CONFIG_HEAT
        return

    def disable_heater(self):
        self.config_reg ^= CONFIG_HEAT
        return

    # fast measure
    def enable_fastmeasure(self):
        self.config_reg |= CONFIG_FAST
        return

    def disable_fastmeasure(self):
        self.config_reg ^= CONFIG_FAST
        return

    def _si7005_measure(self, addr, config):
        time.sleep(0.05)
        error = self.bus.write_byte_data(addr, REG_CONFIG, CONFIG_START|config|self.config_reg)
        if error:
            print 'error = ' + error
            return

        status = STATUS_NOT_READY
        while status & STATUS_NOT_READY:
            status = self.bus.read_byte_data(addr, REG_STATUS)
            if status < 0:
                print 'error, status = ' + status

        error = self.bus.write_byte_data(addr, REG_CONFIG, 0)
        if error:
            print 'error = ' + error
            return

        data = self.bus.read_word_data(addr, REG_DATA)
        if data < 0:
            value = data
        else:
            value = (data & 0x000000FF) * 256 + data & 0x0000FF00

        return value


    # temperature
    def getTemperature(self):
        GPIO.output(self._cs_pin, GPIO.LOW)
        value = self._si7005_measure(SI7005_ADR, CONFIG_TEMPERATURE)
        self.temperature = (((value>>2)*1000)/TEMPERATURE_SLOPE) - (TEMPERATURE_OFFSET*1000)
        self.temperature /= 1000.0
        GPIO.output(self._cs_pin, GPIO.HIGH)

        return self.temperature

    # humidity
    def getHumidity(self):
        GPIO.output(self._cs_pin, GPIO.LOW)
        value = self._si7005_measure(SI7005_ADR, CONFIG_HUMIDITY)
        curve = ((value>>4)*1000)/HUMIDITY_SLOPE - HUMIDITY_OFFSET*1000;
        linear = (curve * SCALAR - (curve*curve*A2)/1000 - curve*A1 - A0 * 1000) / SCALAR
        linear = (linear*SCALAR + (self.temperature - 30000) * ((linear*Q1)/1000 + Q0))/SCALAR
        if linear < 0:
            self.humidity = 0
        elif linear > 100000:
            self.humidity = 100000
        else:
            self.humidity = linear
        self.humidity /= 1000.0
        GPIO.output(self._cs_pin, GPIO.HIGH);

        return self.humidity


if __name__ == '__main__':
    si7005device = si7005(11)
    temp = si7005device.getTemperature()
    print 'temperature = %3.3f' % temp
    humi = si7005device.getHumidity()
    print 'humidity = %3.3f' % humi
