# test_servo_one.py
import time, board, busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=0x40)
pca.frequency = 50  # MG90S @ 50 Hz

s = servo.Servo(
    pca.channels[0],
    min_pulse=500,   # µs
    max_pulse=2500,
    actuation_range=180
)

for ang in (0, 30, 60, 90, 120, 150, 180, 0):
    s.angle = ang
    print("angle ->", ang)
    time.sleep(1.0)

pca.deinit()


# channels 0 and 1  and 3 (thunmb and pointer and ring) close from 0-180
# channels 2 and 4 (middle and pinky), closes from 180-0