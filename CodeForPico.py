#speed values: 0-65535 use scaling in jetson node
#encoders increment every tick jetson converts odom

from machine import Pin, PWM
import sys
import utime

# Motor driver pins (adjust GPIO as needed)
in1 = Pin(16, Pin.OUT)
in2 = Pin(17, Pin.OUT)
enA = PWM(Pin(18))
enA.freq(1000)

in3 = Pin(19, Pin.OUT)
in4 = Pin(20, Pin.OUT)
enB = PWM(Pin(21))
enB.freq(1000)

# Encoder pins
encA1 = Pin(10, Pin.IN, Pin.PULL_UP)
encA2 = Pin(11, Pin.IN, Pin.PULL_UP)
encB1 = Pin(12, Pin.IN, Pin.PULL_UP)
encB2 = Pin(13, Pin.IN, Pin.PULL_UP)

# Encoder counters
left_count = 0
right_count = 0

def left_callback(pin):
    global left_count
    if encA1.value() == encA2.value():
        left_count += 1
    else:
        left_count -= 1

def right_callback(pin):
    global right_count
    if encB1.value() == encB2.value():
        right_count += 1
    else:
        right_count -= 1

encA1.irq(trigger=Pin.IRQ_RISING, handler=left_callback)
encB1.irq(trigger=Pin.IRQ_RISING, handler=right_callback)

def set_motor(left_speed, right_speed):
    # left motor
    if left_speed >= 0:
        in1.high(); in2.low()
    else:
        in1.low(); in2.high()
    enA.duty_u16(min(abs(left_speed), 65535))

    # right motor
    if right_speed >= 0:
        in3.high(); in4.low()
    else:
        in3.low(); in4.high()
    enB.duty_u16(min(abs(right_speed), 65535))

while True:
    line = sys.stdin.readline().strip()
    if line.startswith("SET"):
        try:
            _, _, l, _, r = line.split()
            set_motor(int(l), int(r))
        except:
            pass
    elif line.startswith("GET"):
        print(f"ENC L:{left_count} R:{right_count}")

