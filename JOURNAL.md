---
title: "Saturn Arm"
author: "Kunshpreet"
description: "Development log for a VR-controlled robotic arm project"
---

**Total time spent thus far: 53h**

# May 25-26: Initial Concept and Design
- Created idea
- Pitched concept
- Developed rough Bill of Materials (BOM)
- **Total time spent: 4h**

# May 26-28: Arm Assembly and Design Refinements
- Finished first part of the arm
- Nearly completed second part
- Redesigned bearing holders (original design required impractical assembly)
- Identified needed PCB components:
  - 4 × 3-pin connectors for Servos
  - 1 × 4-pin connector for stepper
  - 1 × 2-pin connector for PSU
  - 3 × MG996R servos
  - M3.5 screws
  - Nema23 23HS5628 stepper + TB6600 Controller
  - SG90 servo
  - LM2596 voltage regulator (for ESP32 compatibility)
  - PSU: [AliExpress link](https://www.aliexpress.com/item/1005005763465796.html)
  - Rotary encoder
- TODO:
  - Finish second arm
  - Complete claw mechanism
  - Finalize bottom part
  - Design camera mount
  ![image](https://github.com/user-attachments/assets/141bd406-e212-4f33-b4ae-24a3a3206a84)
- **Total time spent: 16h**

# May 29: Structural Completion and PCB Design
- Finished all arms + claw assembly
- Improved aesthetic design
- Designed PCB (significant time spent on hole placement)
- Created stepper motor base connection with tight-tolerance square insert
- Current progress: ~60%
- TODO:
  - Complete base assembly
  - Fix 3D printed inserts
  - Finalize BOM
  - Begin coding
 ![image](https://github.com/user-attachments/assets/09dc60fc-7899-4b3b-9948-aef476106900)
- **Total time spent: 6h**

# May 30 - June 1: Component Updates and PCB Revision
- Arm improvements:
  - Added screw mounts to arm holders
  - Designed printable nuts for assembly
  - Created side components
- Major PCB changes:
  - Switched ESP32 → Raspberry Pi Zero W (better camera streaming)
  - Significant time spent updating PCB layout/screw holes
- TODO:
  - Design camera mount
  - Convert to wall-plug PSU
  - Add/remove rotary encoder based on space
  - Update connectors: 2-pin → PSU pins, 4-pin → 6-pin
  - Find placement for TB6600 Controller
![image](https://github.com/user-attachments/assets/777e0cfc-fdc9-4ff2-8b4d-4ecf88cb6e45)
![image](https://github.com/user-attachments/assets/5618c878-ab33-4e4d-97e2-9a86875f2d1e)
- **Total time spent: 10h**

# June 2: Power System and Controller Updates
- Implemented changes:
  - Converted to wall-plug PSU
  - Added second rotary encoder
  - Updated connectors: 2-pin → PSU, 4-pin → 6-pin
  - Replaced TB6600 with space-saving TMC2208 controller
- TODO:
  - Finalize camera mounting solution
  - Complete official BOM
  - Start coding base functionality
![image](https://github.com/user-attachments/assets/8f0385e3-93d8-41d8-a9c7-2c96e0afcc6f)

- **Total time spent: 4h**

# June 2: Github Stuff
- Created Journal.md
- Finished BOM
- TODO:
  - Add Camera to BOM
  - Finalize camera mounting solution
  - Start coding base functionality
  - Start coding VR → Arm Pose in Unity
- **Total time spent: 3h**

# June 2-4: Coding
- Added rotary encoder support
  - Encoder 1 → controls Joint 1
  - Encoder 2 → controls Joint 2
  - Encoder 3 → controls the claw
  - Encoder 4 → controls base (stepper/rotation)
- Implemented inverse kinematics with safety checks
  - Enforces ±90° range from vertical (coded as 90°)
  - Restricts x ∈ [-5.5, 5.5] if y ≥ -2 cm
- Added movement validation system
  - Only moves to reachable coordinates within the arm’s physical limits
  - Displays an error if the point is unreachable
- New command features
  - Supports input like x:5,y:8,z:-2
  - stop command halts all motors immediately
- Maintains stable claw orientation (always facing downward)
- TODO:
  - Finalize camera mounting solution
  - Start coding VR → Arm Pose in Unity
**Total time spent: 10h**
