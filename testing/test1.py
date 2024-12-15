import cv2
import numpy as np
import serial
import time
import struct
from enum import Enum

# Enum for state management
class State(Enum):
    SEARCH_GREEN = 1
    STOP = 2
    GO_TOWARD_GREEN = 3

# Setup serial connection to Arduino
arduino = serial.Serial(port='COM5', baudrate=9600, timeout=0.1)
time.sleep(2)  # Wait for the connection to be established

# Adjusted HSV range for green
lower_green = np.array([35, 100, 50])
upper_green = np.array([85, 255, 255])

# Open the camera
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# PID constants (tune these values for better control)
Kp = 1.5
Ki = 0.02
Kd = 0.5

# PID variables
previous_error = 0
integral = 0

# Initial state
state = State.SEARCH_GREEN

# Function to compute PID
def compute_pid(error):
    global previous_error, integral

    P = Kp * error
    integral += error
    I = Ki * integral
    D = Kd * (error - previous_error)
    previous_error = error

    return P + I + D

def send_command(command, value=0):
    data = struct.pack('cf', command.encode(), value)
    arduino.write(data)
    arduino.flush()
    print(f"Sent command: {command}, value: {value}")

def stop_robot():
    send_command('S')
    print("Green detected, stopping the robot.")

def rotate_robot():
    send_command('R')
    print("Rotating to detect green.")

def process_frame(frame):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

    # Apply an averaging filter to the mask
    green_mask = cv2.blur(green_mask, (5, 5))

    return green_mask

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        green_mask = process_frame(frame)

        green_detected = cv2.countNonZero(green_mask) > 0

        if state == State.SEARCH_GREEN:
            # Start to look for green
            if green_detected:
                stop_robot()
                state = State.STOP
                start_time = time.time()
        elif state == State.STOP:
            # Stop phase
            if time.time() - start_time > 2:
                state = State.GO_TOWARD_GREEN
        elif state == State.GO_TOWARD_GREEN:
            # Go toward green phase
            if green_detected:
                contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(largest_contour)

                    center_x = x + w // 2
                    center_y = y + h // 2

                    frame_center = frame.shape[1] // 2
                    error = (center_x - frame_center) / 10
                    print(f"Error: {error}")

                    pid_output = compute_pid(error)
                    print(f"PID Output: {pid_output}")

                    send_command('P', pid_output)
                    received_pid = arduino.readline().decode().strip()
                    print(received_pid)
                    received_left = arduino.readline().decode().strip()
                    print(received_left)
                    received_right = arduino.readline().decode().strip()
                    print(received_right)

                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        else:
            if state == State.SEARCH_GREEN:
                rotate_robot()

        time.sleep(0.1)
        cv2.imshow('Green Detection with Boundary and Center', frame)

except KeyboardInterrupt:
    print("Stopped by user")

cap.release()
cv2.destroyAllWindows()
