# Final Robotic Project

Welcome to the **Final Robotic Project**! This repository contains all the source code, documentation, and results for the project. The goal of this project is to design and implement a robotic system capable of autonomous navigation, obstacle avoidance, and task execution in a dynamic environment.

---

## 🚀 Features

This project performs the following key tasks:

1. **Grabbing Objects**:  
   - A servo motor controls a crawler to grab a bottle.  
2. **Object Detection**:  
   - A camera detects green-colored objects using computer vision techniques.  
3. **Autonomous Navigation**:  
   - The robot moves to a desired position using Iterative Closest Point (ICP) and LIDAR for localization and mapping.

---

## 📂 Source Code Structure

The project structure is organized as follows:
```
├── Individual_Task_algorithm.ipynb
├── Map Generate.ipynb
├── archives
│   ├── Camera.py
│   ├── PID_Final.ino
│   ├── Servo.txt
│   ├── icpv2.py
│   ├── image.jpg
│   ├── lidar11 (1).csv
│   ├── lidar13 (1).csv
│   ├── lidar31 (1).csv
│   ├── lidarFinal1.csv
│   ├── lidarFinal1012.csv
│   ├── lidarFinal2.csv
│   ├── lidarFinal3.csv
│   └── lidarFinal4.csv
├── bag_files
│   ├── mydata1
│   │   ├── metadata.yaml
│   │   └── mydata1_0.db3
│   ├── mydata2
│   │   ├── metadata.yaml
│   │   └── mydata2_0.db3
│   ├── mydata3
│   │   ├── metadata.yaml
│   │   └── mydata3_0.db3
│   └── mydata4
│       ├── metadata.yaml
│       └── mydata4_0.db3
├── build
│   └── COLCON_IGNORE
├── camera_pid.py
├── components
│   └── servo_sona.ino
├── dataCollect.py
├── exportAstar_and_map.ipynb
├── final_pack.tar
├── finalfinal
│   ├── init_state.py
│   ├── main.py
│   ├── mapfinalfinal.npy
│   ├── path_following.py
│   ├── path_planning.py
│   ├── pathasta.npy
│   └── tune_icp.py
├── finalfinalfinal.ino
├── flipped_pathAsta.npy
├── init_state.py
├── install
│   └── COLCON_IGNORE
├── lidardata
│   ├── build
│   │   └── COLCON_IGNORE
│   ├── install
│   │   ├── COLCON_IGNORE
│   │   ├── _local_setup_util_ps1.py
│   │   ├── _local_setup_util_sh.py
│   │   ├── local_setup.bash
│   │   ├── local_setup.ps1
│   │   ├── local_setup.sh
│   │   ├── local_setup.zsh
│   │   ├── setup.bash
│   │   ├── setup.ps1
│   │   ├── setup.sh
│   │   └── setup.zsh
│   ├── log
│   │   ├── COLCON_IGNORE
│   │   ├── build_2024-12-09_20-45-53
│   │   │   ├── events.log
│   │   │   └── logger_all.log
│   │   ├── build_2024-12-09_21-07-49
│   │   │   ├── events.log
│   │   │   └── logger_all.log
│   │   ├── build_2024-12-09_21-09-47
│   │   │   ├── events.log
│   │   │   └── logger_all.log
│   │   ├── latest -> latest_build
│   │   └── latest_build -> build_2024-12-09_21-09-47
│   └── mydata
│       ├── metadata.yaml
│       └── mydata_0.db3
├── log
│   ├── COLCON_IGNORE
│   ├── build_2024-12-12_01-45-31
│   │   └── logger_all.log
│   ├── latest -> latest_build
│   └── latest_build -> build_2024-12-12_01-45-31
├── main
│   └── main.ino
├── main.ipynb
├── main.py
├── map.npy
├── map_final.npy
├── new_pathviz.py
├── path7.npy
├── pathAsta.npy
├── path_following.py
├── path_planning.py
├── pose_manager.py
├── sona_servo_speedcontrol.py
├── test_morning.py
├── test_pid_simulation.py
├── testing
│   ├── TestCam.py
│   ├── sketch_dec4a
│   │   ├── sonar_and_motor_only.ino
│   │   └── test sona and servo connection.ino
│   ├── sonar_servo.ino
│   ├── sonar_servo.py
│   ├── test1.ino
│   └── test1.py
└── tune_icp.py
```
-- 
## 🛠️ How to Run

  - Clone the repo to your local devices
  - On raspberry pi:
    ```
    python3 finalfinal/init_state.py
    python3 finalfinal/path_planning.py
    python3 finalfinal/path_following.py
    ```

  - On arduino:
    ```
    cd main
    Run the main.ino on your arduino
    ```




