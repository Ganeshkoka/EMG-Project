#minimal I2C + PWM sanity test

import time, board, busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=0x40)   # default address
pca.frequency = 50

ch = pca.channels[0]
# pulse test: sweep duty to see some effect (LED on a channel will visibly change)
for d in (0, 0x2000, 0x4000, 0x6000, 0x8000, 0xA000, 0xC000, 0xE000, 0xFFFF):
    ch.duty_cycle = d
    print("duty:", hex(d))
    time.sleep(0.5)

# leave it off
ch.duty_cycle = 0
pca.deinit()