import cv2
import numpy as np
import serial
import time

# Setup serial connection to Arduino
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
time.sleep(2)  # Wait for the connection to be established

# Adjusted HSV range for green
lower_green = np.array([35, 100, 50])
upper_green = np.array([85, 255, 255])

# Open the camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

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

            # Send the error to Arduino
            arduino.write(f"{error}\n".encode())
            time.sleep(0.05)
            received_error = arduino.readline().decode().strip()
            print("RECEIVED: ", received_error)
            
            # Draw the bounding box and the center point
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        else:
            print("NOT DETECTED")
            # Rotate the robot if no bottle is detected
            arduino.write("1000\n".encode())  # Arbitrary large error to induce rotation
            time.sleep(0.05)

        # Read the PID error sent back from Arduino
        # if arduino.in_waiting > 0:
        #     received_error = arduino.readline().decode().strip()
        #     print(f"Received PID Error: {received_error}")

        # Optionally, add a short delay to make the loop smoother
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by user")

cap.release()
cv2.destroyAllWindows()
