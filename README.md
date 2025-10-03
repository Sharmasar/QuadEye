# QuadEye
# AutoTrack-Quadcopter 

AutoTrack-Quadcopter is a MATLAB/Simulink project that simulates an autonomous drone with a downward-facing camera. The drone detects objects in its environment, converts detections into inertial frames, and uses PID control to navigate toward a selected target.  

This project integrates **Simulink 3D Animation**, **Computer Vision Toolbox**, and a physics-based quadcopter model to create a realistic control and vision pipeline.  

---

##  Features
- Physics-based quadcopter dynamics simulation.  
- Downward-facing virtual camera attached to the drone.  
- Object detection via shape/color segmentation.  
- Manual selection of target object in the camera feed.  
- PID controllers for roll, pitch, yaw, and altitude control.  
- Conversion of detections from camera frame → world frame.  
- Drone autonomously navigates to target object.  

---

##  Dependencies
- MATLAB **R2025a**  
- Simulink  
- Simscape Multibody  
- Simulink 3D Animation  
- Computer Vision Toolbox  
- Control System Toolbox  
- [Quadcopter Drone Model in Simscape](https://www.mathworks.com/matlabcentral/fileexchange/63580-quadcopter-drone-model-in-simscape)  

---

##  Project Structure
- `/models` → Main Simulink models  
- `/scripts` → MATLAB functions for PID + detection logic  
- `/archive` → Legacy prototypes & tests  
- `README.md` → General project info  
- `README_CODE.md` → Code-focused guide  

---

##  Notes
- This is a **simulation-only project** (not tested on real hardware).  
- Object detection currently uses simple shape/color detection; YOLO integration can be added later.  
- All PID control loops are tunable from within Simulink.  

---

##  License
Released under the MIT License. Free to fork, modify, and contribute.  

---

##  Acknowledgments
- MathWorks Simscape Quadcopter Drone Model  
- MATLAB & Simulink Toolboxes  
