from __future__ import print_function  # use python 3 syntax but make it compatible with python 2
from __future__ import division  # changing the division operator
import smbus2
import math
import time


class QMC5883L:
    """Set the data location from the datasheet of QMC5883L (Magnetometer)"""
    DXRA = 0x01  # data output X MSB
    DXRB = 0x00  # data output X LSB
    DYRA = 0x03  # data output Y MSB
    DYRB = 0x02  # data output Y LSB
    DZRA = 0x05  # data output Z MSB
    DZRB = 0x04  # data output Z LSB
    CR = 0x09  # control register
    SR = 0x06  # status register

    MEAS_CONT = 0x01  # continuous measurement
    MEAS_IDLE = 0x00  # idle mode

    # Output data rate
    ODR0 = 0x00  # 10Hz
    ODR1 = 0x01  # 50Hz
    ODR2 = 0x10  # 100Hz
    ODR3 = 0x11  # 200Hz

    # Full scale
    RNG0 = 0x00
    RNG1 = 0x01

    # Over sampling ratio
    OSR0 = 0x00
    OSR1 = 0x01
    OSR2 = 0x10
    OSR3 = 0x11

    def __init__(self, addr=0x0D, bus=1):
        """Set the communication bus and its address"""
        self.addr = addr
        self.bus = smbus2.SMBus(bus)
        self.declination = 0.0
        self.declinationDeg = 0.0
        self.declinationMin = 0.0

    def __str__(self):
        """Set up the the printing message of the corresponding class"""
        ret_str = ""
        axes = self.get_axes()
        ret_str += "Axis X: " + str(axes['x']) + "\n"
        ret_str += "Axis Y: " + str(axes['y']) + "\n"
        ret_str += "Axis Z: " + str(axes['z']) + "\n"

        new_data, data_overflow, data_skip = self.read_status_register()

        ret_str += "New data: " + str(new_data) + "\n"
        ret_str += "Data overflow: " + str(data_overflow) + "\n"
        ret_str += "Data skip: " + str(data_skip) + "\n"

        ret_str += "Declination: " + self.get_declination_string() + "\n"

        ret_str += "Heading: " + self.get_heading_string() + "\n"

        return ret_str

    def set_continuous_mode(self):
        return self.MEAS_CONT

    def set_standby_mode(self):
        return self.MEAS_IDLE

    def reset_period_register(self):
        self.bus.write_byte_data(self.addr, 0x0B, 0x01)

    def set_output_data_rate(self, value=50):
        if value == 10:
            return self.ODR0 << 3
        elif value == 50:
            return self.ODR1 << 3
        elif value == 100:
            return self.ODR2 << 3
        elif value == 200:
            return self.ODR3 << 3
        else:
            return self.ODR1 << 3

    def set_full_scale(self, value=8):
        if value == 2:
            return self.RNG0 << 5
        elif value == 8:
            return self.RNG1 << 5
        else:
            return self.RNG1 << 5

    def set_over_sampling_ratio(self, value=256):
        if value == 512:
            return self.OSR0 << 7
        elif value == 256:
            return self.OSR1 << 7
        elif value == 128:
            return self.OSR2 << 7
        elif value == 64:
            return self.OSR3 << 7
        else:
            return self.OSR1 << 7

    def write_control_register(self, value):
        self.bus.write_byte_data(self.addr, self.CR, value)

    def get_axes(self):
        mag_x = self.read_i2c_word(self.DXRA, self.DXRB)
        mag_y = self.read_i2c_word(self.DYRA, self.DYRB)
        mag_z = self.read_i2c_word(self.DZRA, self.DZRB)

        if mag_x == -4096:
            mag_x = None
        else:
            mag_x = round(mag_x, 4)

        if mag_y == -4096:
            mag_y = None
        else:
            mag_y = round(mag_y, 4)

        if mag_z == -4096:
            mag_z = None
        else:
            mag_z = round(mag_z, 4)

        return {'x': mag_x, 'y': mag_y, 'z': mag_z}

    def read_status_register(self):
        data_status = self.bus.read_byte_data(self.addr, self.SR)

        new_data = data_status & 0x01
        data_overflow = (data_status >> 1) & 0x01
        data_skip = (data_status >> 2) & 0x01

        return new_data, data_overflow, data_skip

    def get_heading(self):
        mag_data = self.get_axes()

        heading_rad = math.atan2(mag_data['y'], mag_data['x'])
        heading_rad += self.declination

        # Correct for reversed heading
        if heading_rad < 0:
            heading_rad += 2 * math.pi

        # Check for wrap and compensate
        if heading_rad > 2 * math.pi:
            heading_rad -= 2 * math.pi

        # Convert to degrees (from radians)
        heading_deg = heading_rad * 180 / math.pi
        degrees = math.floor(heading_deg)
        minutes = round(((heading_deg - degrees) * 60))
        return degrees, minutes

    def set_declination(self, degree, minutes=0):
        self.declinationDeg = degree
        self.declinationMin = minutes
        self.declination = (degree + minutes / 60) * (math.pi / 180)

    def get_heading_string(self):
        (degrees, minutes) = self.get_heading()
        return str(degrees) + "\u00b0 " + str(minutes) + "'"

    def get_declination_string(self):
        return str(self.declinationDeg) + "\u00b0 " + str(self.declinationMin) + "'"

    def read_i2c_word(self, high_register, low_register):
        low_data = self.bus.read_byte_data(self.addr, low_register)
        high_data = self.bus.read_byte_data(self.addr, high_register)

        read_data = (high_data << 8) | low_data

        if read_data >= 0x8000:
            return -((65535 - read_data) + 1)
        else:
            return read_data


if __name__ == '__main__':
    print("Testing QMC5883L\n")
    gy271 = QMC5883L()
    gy271.reset_period_register()
    gy271.write_control_register(gy271.set_continuous_mode() | gy271.set_output_data_rate() |
                                 gy271.set_full_scale() | gy271.set_over_sampling_ratio())
    gy271.set_declination(0, 0)
    while True:
        print(gy271)
        time.sleep(0.10)
