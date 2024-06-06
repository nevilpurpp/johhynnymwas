import RPi.GPIO as GPIO
import time

# GPIO Pin configuration for 5-channel infrared sensor
SENSOR_PINS = [5, 6, 13, 20, 19]  # Update with your actual GPIO pin numbers

# GPIO Pin configuration for motor driver
LEFT_MOTOR_FORWARD_PIN = 17
LEFT_MOTOR_BACKWARD_PIN = 18
RIGHT_MOTOR_FORWARD_PIN = 22
RIGHT_MOTOR_BACKWARD_PIN = 23
LEFT_MOTOR_PWM_PIN = 27  # PWM pin for left motor
RIGHT_MOTOR_PWM_PIN = 24  # PWM pin for right motor

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)

# Setup sensor pins as input
for pin in SENSOR_PINS:
    GPIO.setup(pin, GPIO.IN)

# Setup motor pins as output
GPIO.setup(LEFT_MOTOR_FORWARD_PIN, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_BACKWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_FORWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_BACKWARD_PIN, GPIO.OUT)

# Setup PWM pins
GPIO.setup(LEFT_MOTOR_PWM_PIN, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_PWM_PIN, GPIO.OUT)

# Initialize PWM
left_motor_pwm = GPIO.PWM(LEFT_MOTOR_PWM_PIN, 1000)  # 1kHz frequency
right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_PWM_PIN, 1000)  # 1kHz frequency
left_motor_pwm.start(0)
right_motor_pwm.start(0)

def set_motor_speed(left_speed, right_speed):
    left_motor_pwm.ChangeDutyCycle(left_speed)
    right_motor_pwm.ChangeDutyCycle(right_speed)

def move_forward(left_speed, right_speed):
    GPIO.output(LEFT_MOTOR_FORWARD_PIN, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD_PIN, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD_PIN, GPIO.HIGH)
    set_motor_speed(left_speed, right_speed)

def move_backward(left_speed, right_speed):
    GPIO.output(LEFT_MOTOR_FORWARD_PIN, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD_PIN, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    set_motor_speed(left_speed, right_speed)

def turn_left(speed):
    GPIO.output(LEFT_MOTOR_FORWARD_PIN, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD_PIN, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    set_motor_speed(0, speed)

def turn_right(speed):
    GPIO.output(LEFT_MOTOR_FORWARD_PIN, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    set_motor_speed(speed, 0)

def stop():
    GPIO.output(LEFT_MOTOR_FORWARD_PIN, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    set_motor_speed(0, 0)

def read_sensors():
    return [GPIO.input(pin) for pin in SENSOR_PINS]

# Assign weights to sensors: [leftmost, left, center, right, rightmost]
WEIGHTS = [-2, -1, 0, 1, 2]

def calculate_error(sensor_values):
    return sum(value * weight for value, weight in zip(sensor_values, WEIGHTS))

# PD Controller parameters
KP = 10  # Proportional gain
KD = 9  # Derivative gain
BASE_SPEED = 30  # Base speed for motors (0 to 100)

previous_error = 0
previous_time = time.time()

try:
    while True:
        sensor_values = read_sensors()
        error = calculate_error(sensor_values)
        current_time = time.time()
        dt = current_time - previous_time

        proportional = error
        derivative = (error - previous_error) / dt

        correction = KP * proportional + KD * derivative

        left_speed = BASE_SPEED - correction
        right_speed = BASE_SPEED + correction

        left_speed = max(min(left_speed, 100), 0)  # Clamp between 0 and 100
        right_speed = max(min(right_speed, 100), 0)  # Clamp between 0 and 100

        move_forward(left_speed, right_speed)

        previous_error = error
        previous_time = current_time

        time.sleep(0.01)

except KeyboardInterrupt:
    pass
finally:
    stop()
    left_motor_pwm.stop()
    right_motor_pwm.stop()
    GPIO.cleanup()
