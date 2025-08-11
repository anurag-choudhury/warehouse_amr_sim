# 🚚 AMR-Based Sensing in Smart Warehouse – Simulation (Milestone 1)

This repository contains the simulation environment and Docker setup for **Milestone 1** of the *Autonomous Mobile Robot (AMR)-Based Sensing in Smart Warehouse* project, developed for the **United Nations World Food Programme (WFP)** in collaboration with **FITT IIT Delhi**.

---

## 📌 Project Overview
The AMR is designed to autonomously navigate warehouse aisles, avoid obstacles, and collect **environmental data** (temperature, humidity, CO₂, O₂, airflow, smoke) in real time.  
Data is transmitted via MQTT to a cloud dashboard for warehouse condition monitoring.

- **Payload Capacity:** 100 kg  
- **Dimensions:** 650×550×4100 mm (rod stretched), 650×550×2000 mm (rod retracted)  
- **Simulation Goal:** Validate navigation, SLAM, sensing, and communication pipeline before hardware deployment.

---

## 🛠 Features
- ROS 2 Humble + Gazebo Harmonic environment
- Navigation (Dijkstra/A*, DWA/TEB) via Nav2
- SLAM using Cartographer / slam_toolbox
- Obstacle avoidance with LiDAR + RGB-D
- Environmental sensing data simulation
- MQTT + ROS 2 DDS communications
- Backend-ready hooks for FastAPI/InfluxDB dashboard

---

## 🏗 System Architecture For Hardware Make

**Modules:**
- **Perception:** LiDAR, RGB-D camera, simulated sensors  
- **Localization & Mapping:** SLAM + EKF odometry fusion  
- **Navigation & Control:** Global & local planners, velocity control  
- **Actuation:** Differential drive control  
- **Data Collection:** Periodic environmental sampling  
- **Communication:** Wi-Fi (MQTT, DDS)  
- **Backend:** ROS nodes → MQTT broker → Web dashboard  

**Data Flow:**
1. Sensors → ROS topics  
2. EKF localization → Map + Pose  
3. Planner → Velocity commands  
4. Drive → Differential motor control  
5. MQTT → Cloud database + Dashboard  
6. Optional alerts on threshold breaches

---

## 📦 Docker Environment

The `Dockerfile` installs:

- **ROS 2 Humble** desktop
- **Gazebo Harmonic**
- ROS simulation packages:
  - `ros-humble-gazebo-ros-pkgs`
  - `ros-humble-gazebo-ros2-control`
  - `ros-humble-nav2-bringup`
  - `ros-humble-rtabmap-ros`
  - `ros-humble-ros-gzharmonic`
- Build tools: `colcon`, `cmake`, `build-essential`
- ROS utilities: `python3-colcon-common-extensions`, `python3-rosdep`, `python3-pip`

---

## 🔧 Build & Run Instructions

### 1️⃣ Build the Docker Image
```bash
./build.sh
```


### 2️⃣ Allow X11 Access (Linux host)
```bash 

xhost +local:docker
```
### 3️⃣ Run the Container
```bash
./run.sh
```
### 4️⃣ Build the Workspace (Inside Container)
```bash
colcon build --symlink-install
source install/setup.bash
```
### 5️⃣ Launch the Simulation
```bash
ros2 launch mr_robot_gazebo sim_world.launch.py 
```


