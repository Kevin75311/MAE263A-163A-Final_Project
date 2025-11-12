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
│
├── data/
│ ├── segments/
│ ├── steps/
|
├── docs/
|
├── src/
| │
│ ├── main.py # Entry point
| │
| ├── kinematics/
| │ ├── __init__.py
| │ ├── forward_kinematics.py
| │ ├── inverse_kinematics.py
| │ └── kinematics_test.py
| |
| ├── motor/
| │ ├── __init__.py
| │ ├── motor.py
| │ ├── motor_utils.py
| │ └── motor_test.py
| │
| ├── robot/
| │ ├── __init__.py
| │ └── robot.py
| │
| ├── planner/
| │ ├── __init__.py
| │ └── path_generator.py
| │
| ├── simulator/
| │ ├── __init__.py
| │ └── simulator.py
| │
| ├── visualizer/
| │ ├── __init__.py
| │ └── visualizer.py
│
│
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
conda install dynamixel-controller
```

Then, you should be ready to run the code.