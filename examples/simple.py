import time
from machine import I2C, Pin
import qmi8658c

i2c = I2C(1, scl=Pin(12), sda=Pin(11))
sensor = qmi8658c.QMI8658C(i2c)
print(f"Sensor Revision ID: {sensor.revision_id}")

while True:
    ac = sensor.acceleration
    gy = sensor.gyro
    print(f"Acceleration: X:{ac[0]:.2f}, Y:{ac[1]:.2f}, Z:{ac[2]:.2f} m/s^2")
    print(f"Gyro X:{gy[0]:.2f}, Y:{gy[1]:.2f}, Z:{gy[2]:.2f} rad/s")
    print(f"Temperature: {sensor.temperature:.2f} C")
    print(f"Timestamp: {sensor.timestamp}")

    time.sleep(0.25)