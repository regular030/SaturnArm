<p align="center">
  
  # SaturnArm - A Simple Robotic Arm
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/6ea7d9c7-e56b-4559-b11c-992ef261172b" style="width:400px; height:auto;">
  <img src="https://github.com/user-attachments/assets/7ea0fa0f-be51-418e-8ee4-44945cf4e20b" style="width:330px; height:auto;">
  <img src="https://github.com/user-attachments/assets/1297ab5e-4d0a-43ef-a238-83c6aa38c8c6" style="width:356px; height:auto;">
  <img src="https://github.com/user-attachments/assets/ca28bfae-66ad-4007-b9c0-13a9e16dd829" style="width:450px; height:auto;">
  <img src="https://github.com/user-attachments/assets/63a2db90-d7f2-476f-89ab-c0e46472aff9" style="width:auto; height:264px;">

  <ul>
<p align="center">
  <br>
  BOM: https://docs.google.com/spreadsheets/d/1lf9XG99qlgez3aRTxvFln9WQ-VlTF8MQk1fM6mLnBtQ/edit?usp=sharing
  
  ![image](https://github.com/user-attachments/assets/b0efc2cf-ac89-4011-be87-9e59d9bb2255)
</p>
    
<h2> Why did I Build This?</h2>
<p>
  While watching a NASA Mars rover in action on YouTube, I noticed how every function, including the robotic arm, was controlled through a computer interface. It made me wonder, why not use virtual reality instead, to make the experience more immersive and intuitive?
</p>

# Files 
- Chassis
  - SaturnArm.gcode.3mf
      - G-code to print out the entire arm for the Bambu Lab A1m
  - arm_pcb.step
    - The PCB's 3D model file
  - Full-Arm.f3z
    - Full Fusion/Fusion 360 file for the arm
  - bot
    - All 3D files for the bottom part of the arm    
  - middle
    - All 3D files for the middle part of the arm    
  - top
    - All 3D files for the top part of the arm
- Kicad
  - All PCB files for KiCad 8.0.5
- Firmware
  - All the Firmware files for the Pi Zero 2 W
- Unity
  -  All the Files for the VR to Arm stuff in Unity 2022.3.38f1
  -  DEPRECATED
      
<ul> </ul>
 
#  Instructions

## Building the Firmware on Raspberry Pi Zero 2 W

**1. Install Required Packages**

Open a terminal on your Pi Zero 2 W and run:
```sh
sudo apt update
sudo apt install -y git cmake g++ libopencv-dev libboost-system-dev pigpio
```

**2. Clone the Repository**
```sh
git clone https://github.com/regular030/SaturnArm
cd SaturnArm/Firmware/Arm
```

**3. Enable pigpio Daemon**
```sh
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

**4. Build the Firmware**
```sh
mkdir build
cd build
cmake ..
make -j1
```
The executable `SaturnArm` will be created in the `build` directory.

---

## Calibration

Before using the arm, you must calibrate it:

1. **Remove the Motors:**  
   Remove the motors from the arm.

2. **Run the Firmware:**  
   Start the firmware executable under the build directory:
   ```sh
   sudo ./SaturnArm
   ```

3. **Wait for Program to Run:**  
   Wait for the Program to run, after that wait for the motors to move to 0deg. Once the motors are done moving, make sure the arms are in an upright position before attaching the motors.

---

## Main Functions

- **move_to(x, z):**  
  Move the arm to a specific (x, z) coordinate using inverse kinematics.
     ```sh
   move:(x),(y)
   ```

- **set_servo_angle(servo_number, angle):**  
  Set a specific servo (shoulder, elbow, or claw) to a given angle.
     ```sh
   servo(number):(deg)
   ```

- **test_servos():**  
  Sweep all servos (except base) through their range for testing.

- **emergency_stop():**  
  Immediately stop all motion and disable servos.
  ```sh
   stop
   ```
---

For more details, see the source code in `Firmware/Arm/`.

<ul> </ul>

#  TODO

- Finalize VR Support - Delayed Indefinitely
