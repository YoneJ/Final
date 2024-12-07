import cv2
import numpy as np
import serial
import time
import struct

# Setup serial connection to Arduino
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=0.1)
time.sleep(2)  # Wait for the connection to be established

# Adjusted HSV range for green
lower_green = np.array([35, 100, 50])
upper_green = np.array([85, 255, 255])

""" My idea: Đoạn này mai anh cap màn hình 2 tấm ảnh, 1 tấm xa 1 tấm gần rồi ném vào canva, chấm màu rồi đổi màu RGB -> HSV nhé
 Sâu sắc hơn thì có thread này: https://stackoverflow.com/questions/47483951/how-can-i-define-a-threshold-value-to-detect-only-green-colour-objects-in-an-ima/47483966#47483966
"""
# Open the camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# PID constants (tune these values for better control)
Kp = 1.5  # Proportional gain
Ki = 0.02  # Integral gain
Kd = 0.5  # Derivative gain

# PID variables
previous_error = 0
integral = 0

# Timer variable
start_time = time.time()
detection_phase = True

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
            """
            Lỗi hiện tại em thấy: 
            
                - detection_phase đang làm một cái biến để ngăn màu xanh xuất hiện, ảnh hưởng tới trạng thái sau khi thấy màu xanh
                nhưng boolean khó kiểm soát, dễ bị kẹt loop, không scalable nếu viết thêm code cho task sau,
                điều kiện của boolean lại quá đơn giản nên để thực hiện các task sau boolean sẽ bị conflict
                - (cái này không đọc cũng được) tuy nhiên ưu điểm của boolean là đơn giản nên những task đơn giản mà không yêu cầu
                thêm điều kiện/task sau thì e recommend dùng boolean vì nó không bị overkill nhé
            
            Đề xuất của em: 
            
                - Cách 1 viết enum phases: để switch từ case thấy màu xanh -> case dừng lại -> case bounding box, tìm điểm chính giữa...
                    Code có thể sẽ trông ntn (hoặc hông ai bít): 

                    green_detected = True;
                    
                    if state == "SEARCH GREEN" 
                        thực hiện phase tìm màu xanh
                        if green_detected: 
                            state = "STOP"
                            
                    elif state == "STOP" 
                        dừng lại
                        state == "GO TOWARD GREEN"
                        
                    elif state == "GO TOWARD GREEN"
                    ...
                    
                - Cách 2 là viết finite state machine (FSM)
                    class RobotFSM:
                        def __init__(self):
                            self.state = "SEARCHING"
                    
                        def on_green_detected(self):
                            if self.state == "SEARCHING":
                                self.state = "WAITING"
                                print("Green detected. Switching to WAITING.")
                                self.start_time = time.time()
                    
                        def on_timer_expired(self):
                            if self.state == "WAITING":
                                self.state = "PROCESSING"
                                print("Timer expired. Switching to PROCESSING.")
                    
                        def execute(self):
                            if self.state == "SEARCHING":
                                print("Searching for green.")
                            elif self.state == "WAITING":
                                if time.time() - self.start_time > 2: (đoạn timer em làm theo anh hết)
                                    self.on_timer_expired()
                            elif self.state == "PROCESSING":
                                print("Processing detected green area.")

                - FSM với enum nhìn thì hơi giống nhau nhưng khác ở chỗ là mỗi cách dùng một cách quản lý dữ liệu khác nhau,
                cá nhân em thấy FSM phù hợp hơn vì nó quản lý được nhiều biến độc lập, centralized hơn, điều kiện
                thực hiện và điều kiện nhảy state được quản lý riêng chứ không bị mixed như switch case/enum. 
            """
            if detection_phase:
                # Stop the robot when green is detected for the first time
                arduino.write(struct.pack('f', -1.0))
                print("Green detected, stopping the robot.")
                detection_phase = False
                start_time = time.time()  # Record the time of detection
            else:
                if time.time() - start_time > 2:  # Wait for 2 seconds
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
                        error = (center_x - frame_center) / 10
                        print(f"Error: {error}")

                        # Compute PID to determine robot movement
                        pid_output = compute_pid(error)
                        print(f"PID Output: {pid_output}")

                        # Send the PID output to Arduino to adjust robot movement
                        arduino.write(struct.pack('f', pid_output))
                        received_pid = arduino.readline().decode().strip()
                        print(received_pid)
                        received_left = arduino.readline().decode().strip()
                        print(received_left)
                        received_right = arduino.readline().decode().strip()
                        print(received_right)

                        # Draw the bounding box and the center point
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        else:
            print("Green not detected")
            if detection_phase:
                # Rotate the robot until green is detected
                arduino.write(struct.pack('f', 100.0))  # Arbitrary large error to induce rotation

        # Optionally, add a short delay to make the loop smoother
        time.sleep(0.1)
        # cv2.imshow('Green Detection with Boundary and Center', frame)

except KeyboardInterrupt:
    print("Stopped by user")

cap.release()
cv2.destroyAllWindows()
