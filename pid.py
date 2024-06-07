import RPi.GPIO as GPIO
import time

# GPIO pin assignments for motors
IN1 = 17
IN2 = 18
ENA = 27
IN3 = 22
IN4 = 23
ENB = 24

# GPIO pin assignments for IR sensors
SENSOR1 = 5
SENSOR2 = 6
SENSOR3 = 13
SENSOR4 = 20
SENSOR5 = 19

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)

# Setup motor pins
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Setup sensor pins
GPIO.setup(SENSOR1, GPIO.IN)
GPIO.setup(SENSOR2, GPIO.IN)
GPIO.setup(SENSOR3, GPIO.IN)
GPIO.setup(SENSOR4, GPIO.IN)
GPIO.setup(SENSOR5, GPIO.IN)

# Initialize PWM on ENA and ENB pins
pwmA = GPIO.PWM(ENA, 1000)  # 100 Hz frequency
pwmB = GPIO.PWM(ENB, 1000)  # 100 Hz frequency
pwmA.start(0)  # Start PWM with 0% duty cycle
pwmB.start(0)  # Start PWM with 0% duty cycle

def set_speed(motorA_speed, motorB_speed):
    pwmA.ChangeDutyCycle(motorA_speed)
    pwmB.ChangeDutyCycle(motorB_speed)

def move_forward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
def junction_counter(c):
     sensor_values = sensor_readings()
    cross = 0
   
    while cross < c:
        # Forward movement
        line_following()

        # Check if all IR sensors detect white line
          if sensor_values[0] == 1 and sensor_values[1] == 1 and sensor_values[2] == 1 and sensor_values[3] == 1 and sensor_values[4] == 1:
            # Stop for 100 milliseconds
            stop()
             time.sleep(1.5)
            # Increment cross count
            cross += 1
            # Print cross count
            print(cross)

def sensor_readings():
    
    # Read sensor values (1 means white line detected, 0 means black surface)
    s1 = GPIO.input(SENSOR1)
    s2 = GPIO.input(SENSOR2)
    s3 = GPIO.input(SENSOR3)
    s4 = GPIO.input(SENSOR4)
    s5 = GPIO.input(SENSOR5)
    return [s1, s2, s3, s4, s5]
###################### robotic turns using python script ###############################
def turn_left():
    sensor_values = sensor_readings()
   if sensor_values[0] == 1 and sensor_values[1] == 1 and sensor_values[2] == 1:
    print("Turning left")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    set_speed(BASE_SPEED, BASE_SPEED)
    while sensor_values[2] == 1:
        time.sleep(0.01)
    print("Left turn completed")

def turn_right():
    sensor_values = sensor_readings()
    if sensor_values[0] == 0 and sensor_values[1] == 0 and sensor_values[2] == 1: and sensor_values[3] == 1 and sensor_values[4]== 1:
    print("Turning right")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    set_speed(BASE_SPEED, BASE_SPEED)
    while sensor_values[2] == 1:
        time.sleep(0.01)
    print("Right turn completed")
def turn_u_right():
    #turning right at junctions
    sensor_values = sensor_readings()
    if sensor_values[0] == 1 and sensor_values[1] == 1 and sensor_values[2] == 1: and sensor_values[3] == 1 and sensor_values[4]==1:
        print("at junction")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    set_speed(BASE_SPEED, BASE_SPEED)
    while sensor_values[2] == 1:
        break
        time.sleep(0.01)
    print("Right turn Completed")
    
def about_turn():
    sensor_values = sensor_readings()
    print("Performing about-turn")
    move_backward()
    set_speed(BASE_SPEED, BASE_SPEED)
    time.sleep(1.0)  # Adjust time to complete the about-turn
    while GPIO.input(SENSOR3) == 1:
        time.sleep(0.01)
    move_forward()
    set_speed(BASE_SPEED, BASE_SPEED)
    time.sleep(1.0)  # Adjust time to complete the about-turn
    
# PID Controller parameters
KP = 12  # Proportional gain
KI = 0.1  # Integral gain
KD = 10  # Derivative gain
BASE_SPEED = 50  # Base speed for motors (0 to 100)

previous_error = 0
integral = 0
previous_time = time.time()

def calculate_error(sensor_values):
    # Assign weights to sensors: [leftmost, left, center, right, rightmost]
    WEIGHTS = [-2, -1, 0, 1, 2]
    error = sum(weight * value for weight, value in zip(WEIGHTS, sensor_values))
    return error

def line_following():
    global previous_error, integral, previous_time
    
    while True:
        sensor_values = sensor_readings()
        error = calculate_error(sensor_values)
        # Print the error
        print(f"Sensor values: {sensor_values}, Error: {error}")
        
        current_time = time.time()
        dt = current_time - previous_time

        proportional = error
        integral += error * dt
        derivative = (error - previous_error) / dt if dt > 0 else 0

        correction = (KP * proportional) + (KI * integral) + (KD * derivative)

        left_speed = BASE_SPEED - correction
        right_speed = BASE_SPEED + correction

        # Clamp the speeds between 0 and 100
        left_speed = max(min(left_speed, 100), 0)
        right_speed = max(min(right_speed, 100), 0)

        print(f"Correction: {correction}, Left speed: {left_speed}, Right speed: {right_speed}")

        move_forward()
        set_speed(left_speed, right_speed)

        previous_error = error
        previous_time = current_time
        right_turn()
        junction_counter(2)
        right_u_turn()
        junction_counter(1)
            
        time.sleep(0.01)

try:
    line_following()
   # junction_counter(2)
    #turn_right()
    

except KeyboardInterrupt:
    pass

# Cleanup
stop()
pwmA.stop()
pwmB.stop()
GPIO.cleanup()
