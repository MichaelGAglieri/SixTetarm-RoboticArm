# SixTetarm-RoboticArm v1.0
An open source Custom 6-DOF robotic arm based on Toolboxrobotics project Collaborative Robotic Arm EB-15 — stepper motors nema 17 on RAMPS 1.4, driver a4988, arduino mega, AccelStepper firmware, ROS 2 Jazzy bridge, Webots R2025a simulation with UR3e model.

# WORK IN PRORESS 🚧
**This project is actively under development.** 

The robot is not yet complete — I'm still working on:
- Hardware assembly (J6 joint pending, adaptive)
- Full firmware validation 
- End-to-end testing

**Open to suggestions and collaboration!** Feel free to open issues, propose improvements, or share your ideas. Pull requests welcome!

# Repository structure
SixTetarm-RoboticArm/
├── README.md # Main project documentation
├── LICENSE # Project license (CC BY-NC-SA 4.0)
│
├── 📁 3D Structures/ # 3D printable STL files
│ ├──📁  Planetary_Gear_EBA17/
│ ├── 📁 Planetary_Gear_EB17s/
│ ├── 📁 Collaborative Robot Arm eb-15/
│ ├── 📁 Adaptive Robot Gripper EBG-20/
│ └── Readme.txt
│
├── 📁 Codes/ # Software and firmware
│ ├── 📁 Bridge Ros 2/robot_bridge/
│ ├── 📁 FirmwareROBOT/ # NOT READY
│ ├── 📁 SixTetarm_controller/#webots controller, TESTED, WAIT FOR UPGRADE
│ ├── 📁 Test_stepper_engine_nema/ TESTED, WORK!
│ ├── 📁 Worlds/ # Webots world
│ └── Readme.txt 
│
├── 📁 Sliced Models/
│ ├── Planetary gears EBA17s.3mf
│ ├── Planetary gears EBA17.3mf
│ ├── StrutturaBase.3mf
│ └── Readme.txt
│
└── 📁 Schematics,Datasheets/ # Hardware documentation
│ ├──(schematics, wiring diagrams, PDFs)
│ └── Readme.txt


Project link: https://toolboxrobotics.com/robotic-arm-eb15
License: [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)
Autor: Toolboxrobotics
Complete presentation link of my work : https://prezi.com/p/edit/6eof52t2l8xl/

My contribution is the code, the simulation setup and the tips for 3d print.
