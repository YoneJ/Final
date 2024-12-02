import cv2
import numpy as np
import serial
import time

# Setup serial connection to Arduino
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=0.1)
time.sleep(2)  # Wait for the connection to be established

# Adjusted HSV range for green
lower_green = np.array([35, 100, 50])
upper_green = np.array([85, 255, 255])

# Open the camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

found_green = False

# PID constants (tune these values for better control)
Kp = 3.0  # Proportional gain
Ki = 0.02  # Integral gain
Kd = 0.5  # Derivative gain

# PID variables
previous_error = 0
integral = 0

# Function to compute PID
def compute_pid(error):
    global previous_error, integral

    # Proportional term
    P = Kp * error

    # Integral term
    integral += error
    I = Ki * integral

    # Derivative term
    D = Kd * (error - previous_error)

    # Update previous error for next iteration
    previous_error = error

    # Total PID output
    return P + I + D

try:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Failed to grab frame")
            break
        
        # Convert the frame to HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create a mask for the green color
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

        # Check if green is detected
        if cv2.countNonZero(green_mask) > 0:
            if not found_green:
                # Stop the robot when green is detected for the first time
                arduino.write(b"stop\n")
                print("Green detected, stopping the robot.")
                found_green = True
                time.sleep(1)  # Give the robot some time to stop
            else:
                # Apply morphological transformations to clean up the mask
                kernel = np.ones((15, 15), np.uint8)
                green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        
                # Find contours in the cleaned-up mask
                contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(largest_contour)
        
                    # Calculate the center of the bounding box
                    center_x = x + w // 2
                    center_y = y + h // 2
        
                    # Calculate the error (difference between the center of the frame and the center of the contour)
                    frame_center = frame.shape[1] // 2
                    error = center_x - frame_center
                    print(f"Error: {error}")

                    # Compute PID to determine robot movement
                    pid_output = compute_pid(error)
                    print(f"PID Output: {pid_output}")

                    # Send the PID output to Arduino to adjust robot movement
                    arduino.write(f"{pid_output}\n".encode())

                    # Draw the bounding box and the center point
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        
        else:
            print("Green not detected")
            if not found_green:
                # Rotate the robot until green is detected
                arduino.write("350\n".encode())  # Arbitrary large error to induce rotation
            else:
                # Once green is found, keep moving toward it
                arduino.write("0\n".encode())  # Placeholder to keep moving forward; adjust as needed

        # Optionally, add a short delay to make the loop smoother
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by user")

cap.release()
cv2.destroyAllWindows()
