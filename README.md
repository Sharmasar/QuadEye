# QuadEye
QuadEye is a MATLAB/Simulink project that simulates a fully autonomous drone equipped with a downward-facing camera. The system integrates object detection, coordinate transformation, and closed-loop control to demonstrate how a quadcopter can independently identify and track targets in its environment.

The drone’s camera detects objects on the ground, and these detections are converted from the camera frame to the inertial (world) frame using transformation logic. A set of PID controllers then govern altitude, yaw, pitch, and roll, ensuring smooth and stable navigation toward the detected target.

This project leverages Simulink 3D Animation for visualization, the Computer Vision Toolbox for processing camera data, and a detailed quadcopter dynamics model to simulate realistic flight behavior. Together, these components create an end-to-end autonomous tracking pipeline that is both educational and a foundation for advanced UAV research.

---

##  Features
- Quadcopter simulation in Simulink using physics-based dynamics.
- Downward-facing virtual camera linked to the drone body.
- Object detection via shape/color detection (simple CV pipeline).
- Manual object selection for tracking.
- PID controllers for roll, pitch, yaw, and altitude.
- Conversion between camera frame and world (inertial) frame.
- Autonomous navigation toward selected target.

---

##  Dependencies

To run this project, you need:

- **MATLAB R2025a** (tested version)  
- **Simulink**  
- **Simscape Multibody**  
- **Simulink 3D Animation**  
- **Computer Vision Toolbox**  
- **Control System Toolbox**  
- [Quadcopter Drone Model (Simscape)](https://www.mathworks.com/matlabcentral/fileexchange/63580-quadcopter-drone-model-in-simscape)  

---

##  Project Structure

- `/scripts` → MATLAB functions for camera, detection, and PID logic  
- `/models` → Simulink models, including modified Maneuver Controller  
- `/archive` → All earlier versions and prototypes  
- `README.md` → Project documentation  

---

##  Getting Started

1. Install all dependencies (see above).  
2. Download and add the **Quadcopter Drone Model in Simscape** to your MATLAB path.  
3. Open the main project model:  

4. Run the simulation.  
5. In the visualization window, select an object detected by the drone’s camera.  
6. The quadcopter will use PID control to adjust roll, pitch, yaw, and thrust to move toward the object while stabilizing.

---

##  Notes
- This is a **simulation-only project** (no hardware in the loop).  
- Object detection is based on simple shape/color segmentation for virtual environments.  
- YOLO/Deep Learning can be integrated later, but for now a lightweight pipeline is used to ensure simulation speed.  

---

##  License
This project is released under the MIT License. Feel free to fork, modify, and build upon it.  

---

##  Acknowledgments
- [MathWorks Quadcopter Drone Model](https://www.mathworks.com/matlabcentral/fileexchange/63580-quadcopter-drone-model-in-simscape)  
- MATLAB and Simulink Toolboxes used for modeling, simulation, and visualization.  
