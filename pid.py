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
    cross = 0
   
    while cross < c:
        # Forward movement
        line_following()

        # Check if all IR sensors detect white line
        if GPIO.input(SENSOR1) == 1 and GPIO.input(SENSOR2) == 1 and GPIO.input(SENSOR3) == 1 and GPIO.input(SENSOR4) == 1 and GPIO.input(SENSOR5) == 1:
            # Stop for 100 milliseconds
            stop(100)

            # Increment cross count
            cross += 1

            # Move forward until the outer IR sensors detect black
            while GPIO.input(SENSOR1) == 1 and GPIO.input(SENSOR5) == 1:
                forward()

            # Print cross count
            print(cross)
def detect_junction(sensor_values):
    # Define junction patterns
    T_JUNCTION = [1, 1, 1, 1, 1]
    CROSSROAD = [1, 1, 1, 1, 1]
    LEFT_TURN = [1, 1, 1, 0, 0]
    RIGHT_TURN = [0, 0, 1, 1, 1]
    NO_LINE = [0, 0, 0, 0, 0]

    if sensor_values == T_JUNCTION:
        return "T_JUNCTION"
    elif sensor_values == CROSSROAD:
        return "CROSSROAD"
    elif sensor_values == LEFT_TURN:
        return "LEFT_TURN"
    elif sensor_values == RIGHT_TURN:
        return "RIGHT_TURN"
    elif sensor_values == NO_LINE:
        return "NO_LINE"
    else:
        return "NO_JUNCTION"

def handle_junction(junction_type):
    if junction_type == "T_JUNCTION":
        print("T-junction detected, turning right")
        turn_right()
    elif junction_type == "CROSSROAD":
        print("Crossroad detected, moving forward")
        move_forward()
    elif junction_type == "LEFT_TURN":
        print("Left turn detected, turning left")
        turn_left()
    elif junction_type == "RIGHT_TURN":
        print("Right turn detected, turning right")
        turn_right()
    elif junction_type == "NO_LINE":
        print("No line detected, performing about-turn")
        about_turn()
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
    print("Turning left")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    set_speed(BASE_SPEED, BASE_SPEED)
    while GPIO.input(SENSOR4) == 1:
        time.sleep(0.01)
    print("Left turn completed")

def turn_right():
    print("Turning right")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    set_speed(BASE_SPEED, BASE_SPEED)
    while GPIO.input(SENSOR2) == 1:
        time.sleep(0.01)
    print("Right turn completed")

def about_turn():
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
        

        time.sleep(0.01)

try:
    line_following()
    junction_counter(2)
    
except KeyboardInterrupt:
    pass

# Cleanup
stop()
pwmA.stop()
pwmB.stop()
GPIO.cleanup()
