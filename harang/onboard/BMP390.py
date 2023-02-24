# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_bmp3xx

# I2C setup
i2c = board.I2C()   # uses board.SCL and board.SDA
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)

bmp.sea_level_pressure = 1013.25   #unit: hPa
bmp.pressure_oversampling = 8
bmp.temperature_oversampling = 2

# Get initial altitude
print("Initializing...")
temp = 0.0
for i in range(50):
    temp += bmp.altitude
    time.sleep(0.02)
init_altitude = temp/50
#print("Initial altitude: {:7.3f}m".format(init_altitude))

while True:
    print("Altitude: {:7.3f} m  Temperature: {:5.2f} C".format(bmp.altitude - init_altitude, bmp.temperature))
    time.sleep(0.1)   #unit: s
