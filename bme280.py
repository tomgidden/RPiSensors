#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
A Python class to access BME280 environment sensor. The smbus module is
required.

This is a mash-up of code from

* https://github.com/IDCFChannel/bme280-meshblu-py
  -- Original BME280 code

* https://github.com/kbrownlees/bme280
  -- Tidied up BME280 code

* https://github.com/keiichishima/RPiSensors/blob/master/bmp180.py
  -- Original BMP180 code

with the intention of providing BME280 data through an interface similar
to that of the bmp180.py library
"""

import sensorbase
import argparse
from collections import namedtuple

import smbus

# Oversampling mode
_DEFAULT_ADDRESS = 0x77
OS_MODE_SINGLE = 0b00
OS_MODE_2      = 0b01
OS_MODE_4      = 0b10
OS_MODE_8      = 0b11

class Bme280(sensorbase.SensorBase):

    def __init__(self, bus = None, addr = _DEFAULT_ADDRESS,
                 os_mode = OS_MODE_SINGLE):

        assert(bus is not None)
        assert(addr > 0b000111 and addr < 0b1111000)

        super(Bme280, self).__init__(
            update_callback = self._update_sensor_data)

        self._bus = bus
        self._addr = addr
        self._os_mode = os_mode

        self._osrs_t = 1  # Temperature oversampling x 1
        self._osrs_p = 1  # Pressure oversampling x 1
        self._osrs_h = 1  # Humidity oversampling x 1
        self._mode = 3  # Normal mode
        self._t_sb = 5  # Tstandby 1000ms
        self._filter = 0  # Filter off
        self._spi3w_en = 0  # 3-wire SPI Disable

        self._pressure = None
        self._temperature = None
        self._humidity = None

        self._calibration_h = []
        self._calibration_p = []
        self._calibration_t = []
        self._t_fine = 0.0

        self._ctrl_meas_reg = (self._osrs_t << 5) | (self._osrs_p << 2) | self._mode
        self._config_reg = (self._t_sb << 5) | (self._filter << 2) | self._spi3w_en
        self._ctrl_hum_reg = self._osrs_h

        self._read_calibration_data()

    @property
    def os_mode(self):
        '''Gets/Sets oversampling mode.

        OS_MODE_SINGLE: Single mode.
        OS_MODE_2: 2 times.
        OS_MODE_4: 4 times.
        OS_MODE_8: 8 times.

        '''
        return (self._os_mode)

    @os_mode.setter
    def os_mode(self, os_mode):
        assert(os_mode == OS_MODE_SINGLE
               or os_mode == OS_MODE_2
               or os_mode == OS_MODE_4
               or os_mode == OS_MODE_8)
        self._os_mode = os_mode

    def _reset_calibration(self):
        self._calibration_h = []
        self._calibration_p = []
        self._calibration_t = []
        self._t_fine = 0.0
        self._bus.write_byte_data(self._addr, 0xF2, ctrl_hum_reg)
        self._bus.write_byte_data(self._addr, 0xF4, ctrl_meas_reg)
        self._bus.write_byte_data(self._addr, 0xF5, config_reg)

    def _read_calibration_data(self):
        calib = []

        calib.extend(self._bus.read_i2c_block_data(self._addr, 0x88, 24))
        calib.extend(self._bus.read_i2c_block_data(self._addr, 0xA1, 1))
        calib.extend(self._bus.read_i2c_block_data(self._addr, 0xE1, 7))

        self._calibration_t.append((calib[1] << 8) | calib[0])
        self._calibration_t.append((calib[3] << 8) | calib[2])
        self._calibration_t.append((calib[5] << 8) | calib[4])
        self._calibration_p.append((calib[7] << 8) | calib[6])
        self._calibration_p.append((calib[9] << 8) | calib[8])
        self._calibration_p.append((calib[11] << 8) | calib[10])
        self._calibration_p.append((calib[13] << 8) | calib[12])
        self._calibration_p.append((calib[15] << 8) | calib[14])
        self._calibration_p.append((calib[17] << 8) | calib[16])
        self._calibration_p.append((calib[19] << 8) | calib[18])
        self._calibration_p.append((calib[21] << 8) | calib[20])
        self._calibration_p.append((calib[23] << 8) | calib[22])
        self._calibration_h.append(calib[24])
        self._calibration_h.append((calib[26] << 8) | calib[25])
        self._calibration_h.append(calib[27])
        self._calibration_h.append((calib[28] << 4) | (0x0F & calib[29]))
        self._calibration_h.append((calib[30] << 4) | ((calib[29] >> 4) & 0x0F))
        self._calibration_h.append(calib[31])

        for i in range(1, 2):
            if self._calibration_t[i] & 0x8000:
                self._calibration_t[i] = (-self._calibration_t[i] ^ 0xFFFF) + 1

        for i in range(1, 8):
            if self._calibration_p[i] & 0x8000:
                self._calibration_p[i] = (-self._calibration_p[i] ^ 0xFFFF) + 1

        for i in range(0, 6):
            if self._calibration_h[i] & 0x8000:
                self._calibration_h[i] = (-self._calibration_h[i] ^ 0xFFFF) + 1


    def _compensate_pressure(self, adc_p):
        v1 = (self._t_fine / 2.0) - 64000.0
        v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * self._calibration_p[5]
        v2 += ((v1 * self._calibration_p[4]) * 2.0)
        v2 = (v2 / 4.0) + (self._calibration_p[3] * 65536.0)
        v1 = (((self._calibration_p[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8) + ((self._calibration_p[1] * v1) / 2.0)) / 262144
        v1 = ((32768 + v1) * self._calibration_p[0]) / 32768

        if v1 == 0:
            return 0

        pressure = ((1048576 - adc_p) - (v2 / 4096)) * 3125
        if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
        else:
            pressure = (pressure / v1) * 2

        v1 = (self._calibration_p[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
        v2 = ((pressure / 4.0) * self._calibration_p[7]) / 8192.0
        pressure += ((v1 + v2 + self._calibration_p[6]) / 16.0)
        pressure /= 100

        self._pressure = pressure
        return pressure


    def _compensate_temperature(self, adc_t):
        v1 = (adc_t / 16384.0 - self._calibration_t[0] / 1024.0) * self._calibration_t[1]
        v2 = (adc_t / 131072.0 - self._calibration_t[0] / 8192.0) * (adc_t / 131072.0 - self._calibration_t[0] / 8192.0) * self._calibration_t[2]
        self._t_fine = v1 + v2
        self._temperature = self._t_fine / 5120.0
        return self._temperature


    def _compensate_humidity(self, adc_h):
        var_h = self._t_fine - 76800.0
        if var_h == 0:
            return 0

        var_h = (adc_h - (self._calibration_h[3] * 64.0 + self._calibration_h[4] / 16384.0 * var_h)) * (
            self._calibration_h[1] / 65536.0 * (1.0 + self._calibration_h[5] / 67108864.0 * var_h * (
                1.0 + self._calibration_h[2] / 67108864.0 * var_h)))
        var_h *= (1.0 - self._calibration_h[0] * var_h / 524288.0)

        if var_h > 100.0:
            var_h = 100.0
        elif var_h < 0.0:
            var_h = 0.0

        self._humidity = var_h
        return var_h

    def _update_sensor_data(self):
        data = self._bus.read_i2c_block_data(self._addr, 0xF7, 8)

        self._pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        self._temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        self._hum_raw = (data[6] << 8) | data[7]

        self._temperature = self._compensate_temperature(self._temp_raw)
        self._pressure = self._compensate_pressure(self._pres_raw)
        self._humidity = self._compensate_humidity(self._hum_raw)

    def pressure_and_temperature(self):
        self._update()
        return (
            self._pressure,
            self._temperature
        )

    def all(self):
        self._update()
        return (
            self._humidity,
            self._pressure,
            self._temperature
        )

    def humidity(self):
        self._update()
        return (
            self._humidity
        )

    def pressure(self):
        self._update()
        return (
            self._pressure
        )

    def temperature(self):
        return (
            self._temperature
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--pressure', action='store_true', default=False)
    parser.add_argument('--humidity', action='store_true', default=False)
    parser.add_argument('--temperature', action='store_true', default=False)

    parser.add_argument('--i2c-address', default='0x76')
    parser.add_argument('--i2c-bus', default='1')
    args = parser.parse_args()

    if args.i2c_bus:
        bus = smbus.SMBus(int(args.i2c_bus))
    else:
        bus = smbus.SMBus(1)

    if args.i2c_address:
        addr = int(args.i2c_address, 0)
        bme = Bme280(bus=bus, addr=addr)
    else:
        bme = Bme280(bus=bus)

    (humidity, pressure, temperature) = bme.all()

    if args.pressure:
        print("%7.2f hPa" % pressure)
    if args.humidity:
        print("%7.2f ％" % humidity)
    if args.temperature:
        print("%7.2f ℃" % temperature)

    if not args.pressure and not args.humidity and not args.temperature:
        print("%7.2f hPa" % pressure)
        print("%7.2f ％" % humidity)
        print("%7.2f ℃" % temperature)


if __name__ == '__main__':
    main()
