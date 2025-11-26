# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2023 Taiki Komoda for JINS Inc.
#
# SPDX-License-Identifier: MIT
"""
`qmi8658c`
================================================================================

MicroPython helper library for the QMI8658C 6-DoF Accelerometer and Gyroscope


* Author(s): Taiki Komoda

Implementation Notes
--------------------

**Software and Dependencies:**

* MicroPython firmware for the supported boards: https://micropython.org/download/
"""

# imports

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/jins-tkomoda/CircuitPython_QMI8658C.git"


import struct
from math import radians
from time import sleep

from micropython import const  # type: ignore


_QMI8658C_WHO_AM_I = const(0x0)  # WHO_AM_I register
_QMI8658C_REVISION_ID = const(0x1)  # Divice ID register

_QMI8658C_TIME_OUT = const(0x30)  # time data byte register
_QMI8658C_TEMP_OUT = const(0x33)  # temp data byte register
_QMI8658C_ACCEL_OUT = const(0x35)  # base address for sensor data reads
_QMI8658C_GYRO_OUT = const(0x3B)  # base address for sensor data reads

STANDARD_GRAVITY = 9.80665


class AccRange:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`accelerometer_range`.

    * :py:attr:`AccRange.RANGE_2_G`
    * :py:attr:`AccRange.RANGE_4_G`
    * :py:attr:`AccRange.RANGE_8_G`
    * :py:attr:`AccRange.RANGE_16_G`

    """

    RANGE_2_G = const(0)  # +/- 2g
    RANGE_4_G = const(1)  # +/- 4g
    RANGE_8_G = const(2)  # +/- 8g (default value)
    RANGE_16_G = const(3)  # +/- 16g


class GyroRange:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`gyro_range`.

    * :py:attr:`GyroRange.RANGE_16_DPS`
    * :py:attr:`GyroRange.RANGE_32_DPS`
    * :py:attr:`GyroRange.RANGE_64_DPS`
    * :py:attr:`GyroRange.RANGE_128_DPS`
    * :py:attr:`GyroRange.RANGE_256_DPS`
    * :py:attr:`GyroRange.RANGE_512_DPS`
    * :py:attr:`GyroRange.RANGE_1024_DPS`
    * :py:attr:`GyroRange.RANGE_2048_DPS`

    """

    RANGE_16_DPS = const(0)  # +/- 16 deg/s
    RANGE_32_DPS = const(1)  # +/- 32 deg/s
    RANGE_64_DPS = const(2)  # +/- 64 deg/s
    RANGE_128_DPS = const(3)  # +/- 128 deg/s
    RANGE_256_DPS = const(4)  # +/- 256 deg/s (default value)
    RANGE_512_DPS = const(5)  # +/- 512 deg/s
    RANGE_1024_DPS = const(6)  # +/- 1024 deg/s
    RANGE_2048_DPS = const(7)  # +/- 2048 deg/s


class AccRate:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`accelerometer_rate`.
    Accelerometer low power(LP) mode must be a gyro disabled.

    * :py:attr:`AccRate.RATE_8000_HZ`
    * :py:attr:`AccRate.RATE_4000_HZ`
    * :py:attr:`AccRate.RATE_2000_HZ`
    * :py:attr:`AccRate.RATE_1000_HZ`
    * :py:attr:`AccRate.RATE_500_HZ`
    * :py:attr:`AccRate.RATE_250_HZ`
    * :py:attr:`AccRate.RATE_125_HZ`
    * :py:attr:`AccRate.RATE_62_HZ`
    * :py:attr:`AccRate.RATE_31_HZ`
    * :py:attr:`AccRate.RATE_LP_128_HZ`
    * :py:attr:`AccRate.RATE_LP_21_HZ`
    * :py:attr:`AccRate.RATE_LP_11_HZ`
    * :py:attr:`AccRate.RATE_LP_3_HZ`

    """

    RATE_8000_HZ = const(0)
    RATE_4000_HZ = const(1)
    RATE_2000_HZ = const(2)
    RATE_1000_HZ = const(3)
    RATE_500_HZ = const(4)
    RATE_250_HZ = const(5)
    RATE_125_HZ = const(6)  # (default value)
    RATE_62_HZ = const(7)
    RATE_31_HZ = const(8)
    RATE_LP_128_HZ = const(12)
    RATE_LP_21_HZ = const(13)
    RATE_LP_11_HZ = const(14)
    RATE_LP_3_HZ = const(15)


class GyroRate:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`gyro_rate`.

    * :py:attr:`GyroRate.RATE_G_8000_HZ`
    * :py:attr:`GyroRate.RATE_G_4000_HZ`
    * :py:attr:`GyroRate.RATE_G_2000_HZ`
    * :py:attr:`GyroRate.RATE_G_1000_HZ`
    * :py:attr:`GyroRate.RATE_G_500_HZ`
    * :py:attr:`GyroRate.RATE_G_250_HZ`
    * :py:attr:`GyroRate.RATE_G_125_HZ`
    * :py:attr:`GyroRate.RATE_G_62_HZ`
    * :py:attr:`GyroRate.RATE_G_31_HZ`

    """

    RATE_G_8000_HZ = const(0)
    RATE_G_4000_HZ = const(1)
    RATE_G_2000_HZ = const(2)
    RATE_G_1000_HZ = const(3)
    RATE_G_500_HZ = const(4)
    RATE_G_250_HZ = const(5)
    RATE_G_125_HZ = const(6)  # (default value)
    RATE_G_62_HZ = const(7)
    RATE_G_31_HZ = const(8)


class QMI8658C:  # pylint: disable=too-many-instance-attributes
    """Driver for the QMI8658C 6-DoF accelerometer and gyroscope.

    :param machine.I2C i2c_bus: The I2C bus the device is connected to
    :param int address: The I2C device address. Defaults to :const:`0x68`

    **Quickstart: Importing and using the device**

        .. code-block:: python

            from machine import I2C, Pin
            import qmi8658c

            i2c = I2C(0, scl=Pin(1), sda=Pin(0))
            sensor = qmi8658c.QMI8658C(i2c)

            acc_x, acc_y, acc_z = sensor.acceleration
            gyro_x, gyro_y, gyro_z = sensor.gyro
            temperature = sensor.temperature
    """

    def __init__(self, i2c_bus, address=0x6B) -> None:
        self.i2c = i2c_bus
        self.address = address

        # Shared buffers to avoid heap churn in MicroPython
        self._buf1 = bytearray(1)
        self._buf2 = bytearray(2)
        self._time_buf = bytearray(3)
        self._accel_buf = bytearray(6)
        self._accel_gyro_buf = bytearray(12)

        self._acc_scale = 1
        self._gyro_scale = 1

        device_id = self._read_u8(_QMI8658C_WHO_AM_I)
        if device_id != 0x05:
            raise RuntimeError("Failed to find QMI8658C")

        # REG CTRL1 Enables 4-wire SPI interface, address auto increment, SPI read data big endian
        self._write_u8(0x02, 0b01100000)
        # REG CTRL2 : QMI8658CAccRange_8g  and QMI8658CAccOdr_125Hz
        self.accelerometer_range = AccRange.RANGE_8_G
        sleep(0.01)
        self.accelerometer_rate = AccRate.RATE_125_HZ
        sleep(0.01)
        # REG CTRL3 : QMI8658CGyrRange_512dps and QMI8658CGyrOdr_125Hz
        self.gyro_range = GyroRange.RANGE_512_DPS
        sleep(0.01)
        self.gyro_rate = GyroRate.RATE_G_125_HZ
        sleep(0.01)
        # REG CTRL4 : No magnetometer
        self._write_u8(0x05, 0x00)
        # REG CTRL5 : Disables Gyroscope And Accelerometer Low-Pass Filter
        self._write_u8(0x06, 0x00)
        # REG CTRL6 : Disables Motion on Demand.
        self._write_u8(0x07, 0x00)
        # REG CTRL7 : Enable Gyroscope And Accelerometer
        sleep(0.01)
        self.accelerometer_enable = 1
        sleep(0.1)
        self.gyro_enable = 1
        sleep(0.1)

    @property
    def timestamp(self) -> int:
        """Timestamp from boot up"""
        self.i2c.readfrom_mem_into(self.address, _QMI8658C_TIME_OUT, self._time_buf)
        return (
            self._time_buf[0]
            + (self._time_buf[1] << 8)
            + (self._time_buf[2] << 16)
        )

    @property
    def temperature(self) -> float:
        """Chip temperature"""
        self.i2c.readfrom_mem_into(self.address, _QMI8658C_TEMP_OUT, self._buf2)
        temp = self._buf2[0] / 256 + self._buf2[1]
        return temp

    @property
    def revision_id(self) -> int:
        """Silicon revision value from :data:`_QMI8658C_REVISION_ID`."""
        return self._read_u8(_QMI8658C_REVISION_ID)

    @property
    def acceleration(self):
        """Acceleration X, Y, and Z axis data in :math:`m/s^2`"""
        self.i2c.readfrom_mem_into(self.address, _QMI8658C_ACCEL_OUT, self._accel_buf)
        raw_x, raw_y, raw_z = struct.unpack_from("<hhh", self._accel_buf)

        # setup range dependant scaling
        accel_x = (raw_x / self._acc_scale) * STANDARD_GRAVITY
        accel_y = (raw_y / self._acc_scale) * STANDARD_GRAVITY
        accel_z = (raw_z / self._acc_scale) * STANDARD_GRAVITY

        return (accel_x, accel_y, accel_z)

    @property
    def gyro(self):
        """Gyroscope X, Y, and Z axis data in :math:`rad/s`"""
        self.i2c.readfrom_mem_into(self.address, _QMI8658C_GYRO_OUT, self._accel_buf)
        raw_x, raw_y, raw_z = struct.unpack_from("<hhh", self._accel_buf)

        # setup range dependant scaling
        gyro_x = radians(raw_x / self._gyro_scale)
        gyro_y = radians(raw_y / self._gyro_scale)
        gyro_z = radians(raw_z / self._gyro_scale)

        return (gyro_x, gyro_y, gyro_z)

    @property
    def raw_acc_gyro(self):
        """Raw data extraction"""
        self.i2c.readfrom_mem_into(
            self.address, _QMI8658C_ACCEL_OUT, self._accel_gyro_buf
        )
        return struct.unpack_from("<hhhhhh", self._accel_gyro_buf)

    @property
    def raw_acc_gyro_bytes(self):
        """Raw bytes extraction"""
        self.i2c.readfrom_mem_into(
            self.address, _QMI8658C_ACCEL_OUT, self._accel_gyro_buf
        )
        return tuple(self._accel_gyro_buf)

    @property
    def accelerometer_range(self) -> int:
        """The measurement range of all accelerometer axes. Must be a `AccRange`"""
        return self._read_bits(0x03, 0x70, 4)

    @accelerometer_range.setter
    def accelerometer_range(self, value: int) -> None:
        if (value < 0) or (value > 3):
            raise ValueError("accelerometer_range must be a AccRange")

        if value == AccRange.RANGE_16_G:
            self._acc_scale = 2048
        if value == AccRange.RANGE_8_G:
            self._acc_scale = 4096
        if value == AccRange.RANGE_4_G:
            self._acc_scale = 8192
        if value == AccRange.RANGE_2_G:
            self._acc_scale = 16384

        self._write_bits(0x03, 0x70, 4, value)
        sleep(0.01)

    @property
    def accelerometer_rate(self) -> int:
        """The measurement rate of all accelerometer axes. Must be a `AccRate`"""
        return self._read_bits(0x03, 0x0F, 0)

    @accelerometer_rate.setter
    def accelerometer_rate(self, value: int) -> None:
        if value < 0 or value > 15 or 9 <= value <= 11:
            raise ValueError("accelerometer_rate must be a AccRate")

        if 12 <= value <= 15 and self.gyro_enable == 1:
            raise ValueError("accelerometer low power mode must be a gyro disabled")

        self._write_bits(0x03, 0x0F, 0, value)
        sleep(0.01)

    @property
    def gyro_range(self) -> int:
        """The measurement range of all gyroscope axes. Must be a `GyroRange`"""
        return self._read_bits(0x04, 0x70, 4)

    @gyro_range.setter
    def gyro_range(self, value: int) -> None:
        if (value < 0) or (value > 7):
            raise ValueError("gyro_range must be a GyroRange")

        if value == GyroRange.RANGE_16_DPS:
            self._gyro_scale = 2048
        if value == GyroRange.RANGE_32_DPS:
            self._gyro_scale = 1024
        if value == GyroRange.RANGE_64_DPS:
            self._gyro_scale = 512
        if value == GyroRange.RANGE_128_DPS:
            self._gyro_scale = 256
        if value == GyroRange.RANGE_256_DPS:
            self._gyro_scale = 128
        if value == GyroRange.RANGE_512_DPS:
            self._gyro_scale = 64
        if value == GyroRange.RANGE_1024_DPS:
            self._gyro_scale = 32
        if value == GyroRange.RANGE_2048_DPS:
            self._gyro_scale = 16

        self._write_bits(0x04, 0x70, 4, value)
        sleep(0.01)

    @property
    def gyro_rate(self) -> int:
        """The measurement rate of all gyroscope axes. Must be a `GyroRate`"""
        return self._read_bits(0x04, 0x0F, 0)

    @gyro_rate.setter
    def gyro_rate(self, value: int) -> None:
        if value < 0 or value > 8:
            raise ValueError("gyro_rate must be a GyroRate")
        self._write_bits(0x04, 0x0F, 0, value)
        sleep(0.01)

    @property
    def accelerometer_enable(self) -> int:
        """Enable / disable accelerometer"""
        return self._read_bits(0x08, 0x01, 0)

    @accelerometer_enable.setter
    def accelerometer_enable(self, value: int) -> None:
        if value < 0 or value > 1:
            raise ValueError("accelerometer_enable must be a 0/1")
        self._write_bits(0x08, 0x01, 0, value)
        sleep(0.1)

    @property
    def gyro_enable(self) -> int:
        """Enable / disable gyroscope"""
        return self._read_bits(0x08, 0x02, 1)

    @gyro_enable.setter
    def gyro_enable(self, value: int) -> None:
        if value < 0 or value > 1:
            raise ValueError("gyro_enable must be a 0/1")
        self._write_bits(0x08, 0x02, 1, value)
        sleep(0.1)

    # ---------------------------------------------------------------------
    # Internal helpers
    # ---------------------------------------------------------------------
    def _read_u8(self, register: int) -> int:
        self.i2c.readfrom_mem_into(self.address, register, self._buf1)
        return self._buf1[0]

    def _write_u8(self, register: int, value: int) -> None:
        value &= 0xFF
        try:
            self._buf1[0] = value
            self.i2c.writeto_mem(self.address, register, self._buf1)
        except AttributeError:
            self._buf2[0] = register
            self._buf2[1] = value
            self.i2c.writeto(self.address, self._buf2)

    def _read_bits(self, register: int, mask: int, shift: int) -> int:
        return (self._read_u8(register) & mask) >> shift

    def _write_bits(self, register: int, mask: int, shift: int, value: int) -> None:
        reg = self._read_u8(register)
        reg &= ~mask
        reg |= (value << shift) & mask
        self._write_u8(register, reg)

if __name__ == "__main__":
    import time
    from machine import I2C, Pin  # type: ignore

    i2c = I2C(1, scl=Pin(12), sda=Pin(11))
    sensor = QMI8658C(i2c)
    print(f"Sensor Revision ID: {sensor.revision_id}")

    while True:
        ac = sensor.acceleration
        gy = sensor.gyro
        print(f"Acceleration: X:{ac[0]:.2f}, Y:{ac[1]:.2f}, Z:{ac[2]:.2f} m/s^2")
        print(f"Gyro X:{gy[0]:.2f}, Y:{gy[1]:.2f}, Z:{gy[2]:.2f} rad/s")
        print(f"Temperature: {sensor.temperature:.2f} C")
        print(f"Timestamp: {sensor.timestamp}")

        time.sleep(0.25)