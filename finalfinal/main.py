import cv2
import numpy as np
import serial
import time
import struct
import subprocess
import rclpy
from rclpy.node import Node

arduino = serial.Serial(port='/dev/ttyUSB1', baudrate=115200, timeout=0.1)
time.sleep(2)
lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# PID constants
Kp = 0.0003
Ki = 0.000002
Kd = 0.00008

previous_error = 0
integral_error = 0

start_time = time.time()
frame_last_processed_time = time.time()

class State:

    INIT = "INITSTATE"
    START = "START"
    DETECT_GREEN = "DETECT_GREEN"
    TRACK = "TRACK"
    PLANNING = "PATH_PLANNING"
    FOLLOWPATH = "PATH_FOLLOWING"

current_state = State.START

def compute_pid(error):
        global integral_error
        global previous_error 
        P = error 
        integral_error += error 
        D = error - previous_error
        pid_output = (Kp * P) + (Ki * integral_error) + (Kd * D)
        previous_error = error 

        return pid_output


def transition(new_state):
    global current_state, start_time
    print(f"Transitioning to {new_state}")
    current_state = new_state
    start_time = time.time()


def listen_for_arduino():
        message = arduino.readline().decode('utf-8').strip()
        if message == "done":
            arduino.write("0.0,0.0\n".encode('utf-8'))
            print("Wrapped done, stop robot.")
            transition(State.PLANNING)        

def init_state():
    """
    Ensures init_state.py is running.
    """
    global init_process
    if init_process is None or init_process.poll() is not None:  # Restart if not running
        try:
            init_process = subprocess.Popen(["python3", "init_state.py"])
            print("init_state.py is running.")
        except Exception as e:
            print(f"Error starting init_state.py: {e}")
init_process = None


def plan_path():
    try:
        subprocess.run(["python3", "path_planning.py"], check=True)
        print("path_planning script has been executed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error occurred while running path_planning.py: {e}")

def follow_path():
    try:
        subprocess.run(["python3", "path_following.py"], check=True)
        print("path_following script has been executed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error occurred while running path_following.py: {e}")

try:
    while True:
        current_time = time.time()
        if current_time - frame_last_processed_time >= 0.8:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

            if current_state == State.INIT:
                print("State: INIT")
                init_state()
                transition(State.START)

            elif current_state == State.START:
                print("State: START")
                arduino.write("0.0,0.15\n".encode('utf-8')) #spinning around until seeing the green bottle           
                if cv2.countNonZero(green_mask) > 0:
                    arduino.write("0.0,0.0\n".encode('utf-8'))
                    print("Green detected, stopping the robot.")
                    transition(State.DETECT_GREEN)

            elif current_state == State.DETECT_GREEN: 
                print("State: DETECT_GREEN")
                if time.time() - start_time > 1:
                    transition(State.TRACK)

            elif current_state == State.TRACK:
                print("State: TRACK")
                kernel = np.ones((15, 15), np.uint8)
                green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

                contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(largest_contour)

                    center_x = x + w // 2
                    frame_center = frame.shape[1] // 2

                    error = (frame_center - center_x)
                    print(f"Error: {error}")

                    angular_velocity = compute_pid(error)
                    angular_velocity = np.clip(angular_velocity, -1.0, 1.0)
                    linear_velocity = 0.07
                    print(f"Angular velocity: {angular_velocity}")
                    # Differential steering
                    V_L = linear_velocity -  (angular_velocity*0.189)
                    V_R = linear_velocity + (angular_velocity*0.189)
                    V_L = round(V_L, 3)
                    V_R = round(V_R, 3)
                    print(f"Error: {error}")
                    print(f"Sending V_L: {V_L}, V_R: {V_R}")

                    # # Send velocities over serial to Arduino
                    arduino.write(f"{V_L},{V_R}\n".encode('utf-8'))
                    # # Optionally listen for Arduino's response
                    # if arduino.in_waiting > 0:
                    #     message = arduino.readline().decode().strip()
                    #     print(f"Message from Arduino: {message}")

                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (center_x, y + h // 2), 5, (0, 0, 255), -1)

                else:
                    print("No green object detected.")
                    arduino.write("0.0,0.0\n".encode('utf-8')) 
                    transition(State.START)
                
                listen_for_arduino()

            elif current_state == State.PLANNING:
                print("State: PLANNING")
                
                try:
                    # Run the Node.js path-planning script once
                    result = subprocess.run(
                        ["node", "path_planning.py"],  # Replace with your Node.js script name
                        check=True,                   # Raise an exception if the script fails
                        text=True,                    # Capture output as string
                        stdout=subprocess.PIPE,       # Capture standard output
                        stderr=subprocess.PIPE        # Capture standard error
                    )
                    
                    # Print the output from the Node.js script
                    print(f"Path Planning Output: {result.stdout}")
                    
                    # Process the results if needed
                    # Example: Check if the script returns success
                    if "success" in result.stdout.lower():
                        print("Path planning completed successfully.")
                        transition(State.START)
                    
                except subprocess.CalledProcessError as e:
                    # Handle errors during the execution of the script
                    print(f"Error running path planning: {e.stderr}")
                    # Decide if you want to retry or transition to an error state
        
            elif current_state == State.FOLLOWPATH:
                print("State: FOLLOW_PATH")
                if time.time() - start_time > 2:
                    follow_path()
                    print("Unwrap bottle from pi'")

                    arduino.write("o".encode('utf-8'))
                    transition(State.START)                

            frame_last_processed_time = current_time

except KeyboardInterrupt:
    print("Stopped by user")

cap.release()
cv2.destroyAllWindows()
