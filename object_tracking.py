import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from picamera2 import Picamera2
# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# Motor Driver Pins (L298N)
IN1 = 20 # Pin 38 on Pi (Motor A forward)
IN2 = 16 # Pin 36 on Pi (Motor A backward)
ENA = 21 # Pin 40 on Pi (PWM for Motor A)
IN3 = 27 # Pin 13 on Pi (Motor B forward)
IN4 = 22 # Pin 15 on Pi (Motor B backward)
ENB = 17 # Pin 11 on Pi (PWM for Motor B)
# Ultrasonic Sensor Pins
TRIG = 23
ECHO = 24
# Buzzer Pin
BUZZER_PIN = 10 
# Setup motor pins as output
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
# Setup GPIO pins for ultrasonic sensor
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
# Setup GPIO pin for buzzer
GPIO.setup(BUZZER_PIN, GPIO.OUT)
# Initialize PWM for motors and buzzer
motorA_pwm = GPIO.PWM(ENA, 1000) # Set PWM frequency for Motor A to 1000Hz
motorB_pwm = GPIO.PWM(ENB, 1000) # Set PWM frequency for Motor B to 1000Hz
motorA_pwm.start(0)
motorB_pwm.start(0)
buzzer_pwm = GPIO.PWM(BUZZER_PIN, 1000) # Set PWM frequency for buzzer
buzzer_pwm.start(0) # Start PWM with 0% duty cycle (buzzer off)
# Initialize Picamera2
camera = Picamera2()
camera.configure(camera.create_preview_configuration())
camera.start()
# Function to calculate distance from ultrasonic sensor
def calculate_distance():
 GPIO.output(TRIG, False)
 time.sleep(0.2)
 GPIO.output(TRIG, True)
 time.sleep(0.00001)
 GPIO.output(TRIG, False)
 while GPIO.input(ECHO) == 0:
 pulse_start = time.time()
 while GPIO.input(ECHO) == 1:
 pulse_end = time.time()
 pulse_duration = pulse_end - pulse_start
 distance_from_target = pulse_duration * 17150 # Convert pulse duration to distance
 distance_from_target = round(distance_from_target, 2) # Round to two decimal places
 return distance_from_target
# Function to move motors forward
def move_forward():
 GPIO.output(IN1, GPIO.HIGH)
 GPIO.output(IN2, GPIO.LOW)
 GPIO.output(IN3, GPIO.HIGH)
 GPIO.output(IN4, GPIO.LOW)
 motorA_pwm.ChangeDutyCycle(35) # Set speed to 35
 motorB_pwm.ChangeDutyCycle(35) # Set speed to 35
# Function to move motors backward
def move_backward():
 GPIO.output(IN1, GPIO.LOW)
 GPIO.output(IN2, GPIO.HIGH)
 GPIO.output(IN3, GPIO.LOW)
 GPIO.output(IN4, GPIO.HIGH)
 motorA_pwm.ChangeDutyCycle(35) # Set speed to 35
 motorB_pwm.ChangeDutyCycle(35) # Set speed to 35
# Function to stop motors
def stop_motors():
 GPIO.output(IN1, GPIO.LOW)
 GPIO.output(IN2, GPIO.LOW)
 GPIO.output(IN3, GPIO.LOW)
 GPIO.output(IN4, GPIO.LOW)
 motorA_pwm.ChangeDutyCycle(0)
 motorB_pwm.ChangeDutyCycle(0)
# Function to detect green objects in the frame
def detect_object(frame):
 hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 lower_green = np.array([40, 100, 100]) # Lower bound for green
 upper_green = np.array([80, 255, 255]) # Upper bound for green
 mask = cv2.inRange(hsv_frame, lower_green, upper_green)
 contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
 return len(contours) > 0 # Return True if green object detected, else False
# Main loop for object tracking, motor control, and buzzer activation
try:
 while True:
 # Read distance from ultrasonic sensor
 distance_from_target = calculate_distance()
 
 # Add a small delay to stabilize reactions
 time.sleep(0.5) # Delay after distance measurement (adjust as needed)
 
 # Capture frame from camera
 frame = camera.capture_array()
 
 # Check for green object in the frame
 object_detected = detect_object(frame)
 print(f"Distance from target: {distance_from_target} cm")
 if object_detected:
 if distance_from_target < 4.2:
 print("Moving forward for 0.5 seconds")
 move_forward()
 buzzer_pwm.ChangeDutyCycle(50) # Set duty cycle to 50% for quieter buzzer
 time.sleep(0.5) # Move for 0.5 seconds
 stop_motors() # Stop to recheck distance
 elif distance_from_target > 10.9:
 print("Moving backward for 0.5 seconds")
 move_backward()
 buzzer_pwm.ChangeDutyCycle(50) # Set duty cycle to 50% for quieter buzzer
 time.sleep(0.5) # Move for 0.5 seconds
 stop_motors() # Stop to recheck distance
 # Turn off the buzzer and stop motors if within range
 elif 4.2 <= distance_from_target <= 10.9:
 print("Within range. Stopping motors and deactivating buzzer")
 stop_motors()
 buzzer_pwm.ChangeDutyCycle(0) # Turn buzzer off
 else:
 print("No green object detected, stopping motors and deactivating buzzer")
 stop_motors() # Stop motors
 buzzer_pwm.ChangeDutyCycle(0) # Turn buzzer off
 time.sleep(0.5) # Delay for stability
except KeyboardInterrupt:
 print("Program stopped")
finally:
 stop_motors()
 buzzer_pwm.ChangeDutyCycle(0) # Ensure buzzer is off
 GPIO.cleanup()
