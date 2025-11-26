import time
from machine import I2C, Pin  # type: ignore
from qmi8658c import QMI8658C

i2c = I2C(0)

# For specific pins, uncomment and modify the following line:
#i2c = I2C(1, scl=Pin(12), sda=Pin(11))
sensor = QMI8658C(i2c)
print(f"Sensor Revision ID: {sensor.revision_id}")

while True:
    acc_g = sensor.acceleration
    acc_ms2 = sensor.acceleration_m_s2
    ang = sensor.angular_velocity
    print(
        "🏃 Acceleration (g): "
        f"X:{acc_g[0]:.2f}, Y:{acc_g[1]:.2f}, Z:{acc_g[2]:.2f}"
    )
    print(
        "🏃 Acceleration (m/s^2): "
        f"X:{acc_ms2[0]:.2f}, Y:{acc_ms2[1]:.2f}, Z:{acc_ms2[2]:.2f}"
    )
    print(
        "🌀 Angular velocity: "
        f"X:{ang[0]:.2f}, Y:{ang[1]:.2f}, Z:{ang[2]:.2f} rad/s"
    )
    print(f"🌡️ Temperature: {sensor.temperature:.2f} C")
    print(f"⏱️ Timestamp: {sensor.timestamp}")
    time.sleep(0.1)