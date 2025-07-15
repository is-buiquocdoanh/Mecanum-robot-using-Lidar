import RPi.GPIO as GPIO
import time

# Khai báo các chân encoder (theo GPIO BCM)
encoder_pins = {
    'FL': {'A': 18, 'B': 17},
    'FR': {'A': 27, 'B': 24},
    'RL': {'A': 15, 'B': 22},
    'RR': {'A': 23, 'B': 25}
}

counts = {wheel: 0 for wheel in encoder_pins}

def make_callback(wheel):
    def callback(channel):
        a = GPIO.input(encoder_pins[wheel]['A'])
        b = GPIO.input(encoder_pins[wheel]['B'])
        if a == b:
            counts[wheel] += 1
        else:
            counts[wheel] -= 1
        print(f"{wheel}: {counts[wheel]}")
    return callback

# Cấu hình GPIO
GPIO.setmode(GPIO.BCM)
for wheel, pins in encoder_pins.items():
    GPIO.setup(pins['A'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(pins['B'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(pins['A'], GPIO.BOTH, callback=make_callback(wheel), bouncetime=1)

print("Đang đọc encoder 4 bánh... Nhấn Ctrl+C để dừng")

try:
    while True:
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Dừng")
finally:
    GPIO.cleanup()
