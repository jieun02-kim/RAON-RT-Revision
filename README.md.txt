# RAON-RT

**RAON-RT** (Real-time Architecture for Open, Networked robotic systems)  
A modular and extensible framework designed to **standardize software across heterogeneous robots**, supporting seamless integration of various **actuators**, **sensors**, and **control modules** in **real-time** environments.

---

## 🔧 Key Features

- ⚙️ **Real-Time Control Layer** for consistent and deterministic behavior
- 🔌 **Hardware Abstraction Layer** for sensors and actuators
- 🔄 **ROS2-Compatible Middleware** with real-time nodelets
- 📦 **Modular Plugin System** for motion, perception, and communication
- 📡 **Multi-Robot & Distributed System Support**
- 🧠 **AI/Simulation Integration Ready**

---

## 📐 Architecture Overview

```mermaid
graph TD
    A[RAON-RT Core]
    A --> B[RT Kernel Interface]
    A --> C[Hardware Abstraction Layer]
    A --> D[Middleware (ROS2/Custom)]
    C --> E[Actuator Plugins]
    C --> F[Sensor Plugins]
    D --> G[Real-Time Nodelets]
    D --> H[Simulation / AI Hooks]
    A --> I[System Config & Management]


RAON-RT: A Real-Time Architecture for Open, Networked Robotic Systems
Toward Standardized Integration of Heterogeneous Actuators and Sensors