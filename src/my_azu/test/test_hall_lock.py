import Jetson.GPIO as GPIO
import time

PIN_7 = 7
PIN_15 = 15

GPIO.setmode(GPIO.BOARD)

GPIO.setup(PIN_7, GPIO.IN)
GPIO.setup(PIN_15, GPIO.IN)

prev_7 = GPIO.input(PIN_7)
prev_15 = GPIO.input(PIN_15)

print("Monitoring Hall sensors on PIN 7 and PIN 15...")

try:
    while True:
        state_7 = GPIO.input(PIN_7)
        state_15 = GPIO.input(PIN_15)

        # ===== PIN 7 =====
        if prev_7 == 0 and state_7 == 1:
            print("PIN 7 -> 🟢 MAGNET REMOVED")

        elif prev_7 == 1 and state_7 == 0:
            print("PIN 7 -> 🧲 MAGNET DETECTED")

        # ===== PIN 15 =====
        if prev_15 == 0 and state_15 == 1:
            print("PIN 15 -> 🟢 MAGNET REMOVED")

        elif prev_15 == 1 and state_15 == 0:
            print("PIN 15 -> 🧲 MAGNET DETECTED")

        prev_7 = state_7
        prev_15 = state_15

        time.sleep(0.01)

except KeyboardInterrupt:
    GPIO.cleanup()