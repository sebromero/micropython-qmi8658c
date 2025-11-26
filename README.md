# 📦 MicroPython QMI8658C Driver

MicroPython helper library for the QMI8658C 6-DoF accelerometer and gyroscope. It exposes a lightweight API for reading motion, angular velocity, and temperature data while providing fine control over sensor configuration, power, and sampling behavior. 

This driver was adapted from the [CircuitPython QMI8658C driver](https://github.com/tkomde/CircuitPython_QMI8658C) by Taiki Komoda.

## ✨ Features

- 📈 **High-fidelity motion data** – Read acceleration in `g` or `m/s²`, angular velocity in `rad/s`, temperature, timestamps, and raw register dumps.
- 🎚️ **Configurable sensor ranges** – Select accelerometer (±2 g to ±16 g) and gyroscope (±16 dps to ±2048 dps) ranges to match your application.
- 🕒 **Flexible output rates** – Tune accelerometer and gyroscope data rates, including low-power accelerometer modes.
- 🔌 **Runtime power control** – Enable or disable individual sensing blocks to conserve energy.
- 📂 **Example included** – `examples/basic_example.py` demonstrates sensor bring-up and streaming data prints.

## 📖 Documentation

For API details, usage notes, and an end-to-end script, see [`examples/basic_example.py`](examples/basic_example.py). The source docstrings inside [`src/qmi8658c.py`](src/qmi8658c.py) describe every property and helper in detail.

Example usage:

```python
from machine import I2C, Pin
from qmi8658c import QMI8658C

i2c = I2C(0)
sensor = QMI8658C(i2c)
acc_x, acc_y, acc_z = sensor.acceleration
gyro_x, gyro_y, gyro_z = sensor.angular_velocity
print("Acceleration (g):", acc_x, acc_y, acc_z)
print("Gyroscope (rad/s):", gyro_x, gyro_y, gyro_z)
```

## ✅ Supported Boards

Any MicroPython board that provides an I2C bus (e.g., Raspberry Pi Pico, ESP32, STM32-based boards, Arduino boards running MicroPython) works with this driver. Connect the QMI8658C sensor to the board's I2C pins (plus 3V3 and GND).

## ⚙️ Installation

Install directly onto a board using `mpremote` + `mip`:

```bash
mpremote mip install github:sebromero/micropython-qmi8658c
```


## 🧑‍💻 Developer Installation

Clone the repo and iterate without flashing files repeatedly:

```bash
git clone https://github.com/sebromero/micropython-qmi8658c.git
cd micropython-qmi8658c
mpremote connect mount src run ./examples/basic_example.py
```

If `mpremote` cannot auto-detect your board, specify its serial ID:

```bash
mpremote connect id:SERIAL_NUMBER mount src run ./examples/basic_example.py
```

Use `mpremote connect list` to discover the correct ID.

## 🐛 Reporting Issues

Found a bug or missing feature? Open an issue in the [GitHub tracker](https://github.com/sebromero/micropython-qmi8658c/issues).

## 📕 Further Reading

- [QMI8658C Datasheet (QST)](https://qstcorp.com/upload/pdf/202202/QMI8658C%20datasheet%20rev%200.9.pdf)
- [MicroPython `mpremote` docs](https://docs.micropython.org/en/latest/reference/mpremote.html)

## 💪 Contributing

Pull requests are welcome! Please discuss substantial changes in an issue before submitting a PR so we can align on direction.

## 🤙 Contact

Questions or feedback? Create an issue on this repository—it's the fastest way to reach the maintainer.
