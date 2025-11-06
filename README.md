# MAE263A / C263A — Robotics Final Project

## Long Exposure Photo Art Robot
(some cool introduction...)

---

## Team Members

| Name | Role |
|------|------|
| **Alex Lie** | Electrical Systems |
| **Andrew Tan** | Mechanical Design |
| **John Meshinsky** | Mechanical Design |
| **Yonghao Huo** | Mechanical Design |
| **Robbie Marlow** | Control & Kinematics |
| **John Chung** | Integration & Proof of Concept |
| **Kevin Huang** | Software & Simulation |


---

## System Components
- **6 × MX-28AR Servo Motors**  
- **Slider-Crank Mechanism** for prismatic motion  

---

## Features

- PRRR robot
- Python-based path planning and visualization tools  
- Preset trajectories (e.g., stars, faces, text outlines)  
- Real-time motion control and exposure synchronization  

---

## Software Architecture (Python)

The Python software stack handles **trajectory generation**, **simulation**, and **inverse kinematics** for motion control.

### File Structure
```
MAE263A-163A-Final_Project/
│
├── README.md
├── main.py # Entry point
│
├── data/
│ ├── segments/
│ ├── steps/
|
├── docs/
|
├── src/
│ ├── kinematics/
│ │ ├── forward_kinematics.py
│ │ ├── inverse_kinematics.py
│ │
│ ├── planning/
│ │ ├── path_planning.py
│ │ ├── presets.py
│ │
│ ├── simulation/
│ │ ├── simulation.py
│ │ ├── visualization.py
│ │
│ ├── control/
│ │ ├── motor_interface.py
│ │ ├── controller.py
│ │
│ ├── utils/
│ │ ├── geometry_utils.py
|
|
└── tests/ # Motion logs and debug data
```


---

## Setup

### Installation
Please install [conda](https://docs.conda.io/projects/conda/en/stable/user-guide/install/index.html) beforehand

```bash
git clone https://github.com/Kevin75311/MAE263A-163A-Final_Project.git
cd MAE263A-163A-Final_Project
conda create --name=pyoccenv python=3.10
conda activate pyoccenv
conda install -c conda-forge pythonocc-core=7.9.0
conda install numpy matplotlib
```

Then, you should be ready to run the code.