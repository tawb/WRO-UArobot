# open_challenge_robot_main.py
"""
Starter control program for the WRO 2025 Future‑Engineers Open Challenge (3‑lap, no‑wall‑touch run).
Hardware stack assumed:
    • Raspberry Pi 4 (Python 3.11) – high‑level vision + strategy
    • MCU (Arduino Nano / RP2040 etc.) – low‑level PWM to motors, encoders, optional IMU fusion
    • 2× HC‑SR04 (or other narrow‑beam ultrasonic) mounted left & right
    • Pi Camera v2 or HQ (forward‑facing, tilted ~15° down)
    • 1× TCS34725 colour sensor board facing the mat between the wheels

Serial protocol (Pi → MCU) –  at 115200 baud, newline‑terminated ASCII:
    "V,<left_speed>,<right_speed>"      # speeds −100…100 (% PWM)
    "S"                                  # stop / motor off

Serial protocol (MCU → Pi):
    "E,<left_ticks>,<right_ticks>,<heading_deg>"  # encoder delta + IMU heading
(Modify as needed.)

Install deps on the Pi:
    sudo apt install python3-opencv python3-numpy python3-serial pigpio python3-pigpio
    pip install smbus2 RPi.GPIO adafruit-circuitpython-tcs34725

Run:
    python3 open_challenge_robot_main.py

Press the physical Start button *before* execution; the script waits for a GPIO‑falling edge, then arms.
"""
from __future__ import annotations
import time
import threading
import serial
import cv2 as cv
import numpy as np
import pigpio
from smbus2 import SMBus
import board  # type: ignore
import busio  # type: ignore
import adafruit_tcs34725  # type: ignore
from collections import deque

# ------------- CONFIG ---------------------------------------------------------
SERIAL_PORT = "/dev/ttyUSB0"      # adjust to your adapter
BAUD_RATE = 115200
FRAME_WIDTH = 640                # camera capture width
FRAME_HEIGHT = 480

LEFT_TRIG = 23                   # GPIO pins for HC‑SR04 left
LEFT_ECHO = 24
RIGHT_TRIG = 27                  # HC‑SR04 right
RIGHT_ECHO = 22

START_BUTTON = 17                # GPIO for hardware Start button (active‑low)
STOP_AT_LAP = 3                  # exactly 3 laps
MAX_RUNTIME = 175                # seconds safety cutoff (< 3‑min limit)

# PID gains (tune!)
KP = 0.8
KI = 0.0
KD = 0.12
TARGET_MID = 0.0                 # desired offset (m) from track centre

# Lap colour thresholds (raw sensor 0‑255)
BLUE_THRESH = 120
ORANGE_THRESH = 150

# ----------------------------------------------------------------------------

class UltrasonicPair:
    """Non‑blocking distance read using pigpio callbacks."""

    def __init__(self, pi: pigpio.pi, trig: int, echo: int):
        self.pi = pi
        self.trig = trig
        self.echo = echo
        self.distance_cm: float = 200.0  # initialise far
        self._high_tick = None
        pi.set_mode(trig, pigpio.OUTPUT)
        pi.set_mode(echo, pigpio.INPUT)
        pi.callback(echo, pigpio.EITHER_EDGE, self._cb)
        self._lock = threading.Lock()

    def _cb(self, gpio, level, tick):
        if level == 1:  # rising
            self._high_tick = tick
        elif level == 0 and self._high_tick is not None:  # falling
            dt = pigpio.tickDiff(self._high_tick, tick)
            d = dt / 58.0  # µs to cm /2 and speed of sound
            with self._lock:
                self.distance_cm = d
            self._high_tick = None

    def trigger(self):
        self.pi.gpio_trigger(self.trig, 10)  # 10 µs pulse

    def read(self) -> float:
        with self._lock:
            return self.distance_cm


class ColourSensor:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_tcs34725.TCS34725(i2c)
        # Enable built‑in LED if board supports
        self.sensor.integration_time = 100  # ms
        self.sensor.gain = 4

    def read(self) -> tuple[int, int, int]:
        r, g, b = self.sensor.color_rgb_bytes
        return r, g, b

    def is_blue_or_orange(self) -> str | None:
        r, g, b = self.read()
        if b > BLUE_THRESH and b > r and b > g:
            return "blue"
        if r > ORANGE_THRESH and g > ORANGE_THRESH and b < 80:
            return "orange"
        return None


class Vision:
    """Detect lane centre offset using simple colour thresholding."""

    def __init__(self):
        self.cap = cv.VideoCapture(0)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    def get_offset(self) -> float:
        ret, frame = self.cap.read()
        if not ret:
            return 0.0
        # Crop lower third
        roi = frame[int(FRAME_HEIGHT * 0.6):, :]
        hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
        # White lane lines mask
        mask_white = cv.inRange(hsv, (0, 0, 200), (180, 30, 255))
        # Compute centre of white pixels
        M = cv.moments(mask_white)
        if M["m00"] > 1000:
            cx = int(M["m10"] / M["m00"])
            centre_offset_px = cx - roi.shape[1] // 2
            # Convert pixels to metres via empirical factor
            metres_per_px = 0.003  # adjust after calibration
            return centre_offset_px * metres_per_px
        return 0.0


class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_err = 0.0
        self.i_term = 0.0
        self.prev_time = time.perf_counter()

    def update(self, error):
        now = time.perf_counter()
        dt = now - self.prev_time
        de = (error - self.prev_err) / dt if dt else 0.0
        self.i_term += error * dt

        output = self.kp * error + self.ki * self.i_term + self.kd * de

        self.prev_err = error
        self.prev_time = now
        return output


class Robot:
    def __init__(self):
        self.pi = pigpio.pi()
        self.ultra_left = UltrasonicPair(self.pi, LEFT_TRIG, LEFT_ECHO)
        self.ultra_right = UltrasonicPair(self.pi, RIGHT_TRIG, RIGHT_ECHO)
        self.vision = Vision()
        self.colour = ColourSensor()
        self.pid = PID(KP, KI, KD)
        self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        self.lap_count = 0
        self._colour_history: deque[str] = deque(maxlen=5)
        self.running = False

    def wait_for_start(self):
        self.pi.set_mode(START_BUTTON, pigpio.INPUT)
        self.pi.set_pull_up_down(START_BUTTON, pigpio.PUD_UP)
        print("Waiting for start button…")
        while self.pi.read(START_BUTTON) == 1:
            time.sleep(0.05)
        # Debounce
        time.sleep(0.3)
        self.running = True
        print("Go!")

    def _send_speed(self, left: float, right: float):
        msg = f"V,{int(left)},{int(right)}\n"
        self.serial.write(msg.encode())

    def _stop(self):
        self.serial.write(b"S\n")

    def update_lap_counter(self):
        colour = self.colour.is_blue_or_orange()
        if colour:
            self._colour_history.append(colour)
            if list(self._colour_history) == ["blue", "orange", "blue", "orange"]:
                self.lap_count += 1
                self._colour_history.clear()
                print(f"Lap {self.lap_count} complete")

    def tick_ultrasonic(self):
        # Trigger both sensors every 60 ms in a background thread
        while self.running:
            self.ultra_left.trigger()
            self.ultra_right.trigger()
            time.sleep(0.06)

    def run(self):
        self.wait_for_start()
        thread = threading.Thread(target=self.tick_ultrasonic, daemon=True)
        thread.start()

        t0 = time.time()
        base_speed = 60  # % PWM — tune this

        while self.running:
            if time.time() - t0 > MAX_RUNTIME:
                print("Runtime limit hit, stopping.")
                break

            # Sensor reads
            left_d = self.ultra_left.read() / 100.0  # convert cm→m
            right_d = self.ultra_right.read() / 100.0
            cam_offset = self.vision.get_offset()

            # Fuse (simple average, could weight by variance)
            centre_err = (right_d - left_d) / 2.0  # + cam_offset*0.5
            centre_err += cam_offset

            correction = self.pid.update(centre_err - TARGET_MID)
            left_speed = base_speed - correction * 40
            right_speed = base_speed + correction * 40

            # Clip speeds
            left_speed = max(min(left_speed, 100), -100)
            right_speed = max(min(right_speed, 100), -100)

            # Command motors
            self._send_speed(left_speed, right_speed)

            # Lap logic
            self.update_lap_counter()
            if self.lap_count >= STOP_AT_LAP:
                print("Done, 3 laps achieved!")
                break

            # Wall‑touch fail‑safe
            if left_d < 0.03 or right_d < 0.03:  # < 3 cm means contact
                print("Wall! aborting run.")
                break

        self._stop()
        self.running = False
        time.sleep(0.5)


if __name__ == "__main__":
    try:
        Robot().run()
    except KeyboardInterrupt:
        print("Interrupted by user")
