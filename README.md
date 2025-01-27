# Towards Safe Motion Planning for Industrial Human-Robot Interaction

Code Repository for the Paper: "Towards Safe Motion Planning for Industrial 🤖🤝"

---

## Project Overview

This repository provides the 📦 supporting the implementation and experimentation of safe motion planning for industrial 🤖🤝. It includes core 🧩 for motion control, sensing, simulation, and reinforcement 🧠 tasks. The following sections describe the project structure and offer detailed insights into the functionality of each 🧩.

---

## 📂 Folder Structure

```
/Assets
  ├── 📜 Scripts
  │     ├── 🛠 Control
  │     │     ├── ArticulationJointController.cs
  │     │     ├── RobotController.cs
  │     │     ├── PincherController.cs
  │     │     └── PincherFingerController.cs
  │     ├── 🔍 Detection
  │     │     ├── TouchCubeDetector.cs
  │     │     ├── ObstacleTouchDetector.cs
  │     │     ├── TableTouchDetector.cs
  │     │     └── TouchBottomDetector.cs
  │     ├── 🎮 Simulation
  │     │     ├── TablePositionRandomizer.cs
  │     │     └── MovementControl.cs
  │     ├── 🤖 Agents
  │           ├── MoveCubeAgent.cs
  │           ├── FinalAgent.cs
  │           └── TouchGrabAgent.cs
```

---

## Core 🧩

### 🛠 Control 🧩

#### **RobotController.cs**
- **✨ Functionality**: Controls 🤖 arm joint rotation, retrieves joint states, and sets 🎯 positions.
- **⚙️ Hardware Dependency**: Requires kinematic alignment with the hardware model.
- **🗂 Path**: `Scripts/Control/RobotController.cs`

#### **PincherController.cs**
- **✨ Functionality**: Manages gripper movements and provides real-time feedback for precise object handling.
- **⚙️ Hardware Dependency**: Customizable gripping speed and force.
- **🗂 Path**: `Scripts/Control/PincherController.cs`

#### **ArticulationJointController.cs**
- **✨ Functionality**: Implements joint control logic, supporting dynamic adjustments.
- **⚙️ Hardware Dependency**: Requires joint-specific parameter configuration.
- **🗂 Path**: `Scripts/Control/ArticulationJointController.cs`

---

### 🔍 Sensing and Detection 🧩

#### **TouchCubeDetector.cs**
- **✨ Functionality**: Detects gripper contact with 🧊 for 🧠 rewards.
- **⚙️ Hardware Dependency**: Precision depends on sensor calibration.
- **🗂 Path**: `Scripts/Detection/TouchCubeDetector.cs`

#### **ObstacleTouchDetector.cs**
- **✨ Functionality**: Identifies collisions between the 🤖 and obstacles for penalty logic.
- **⚙️ Hardware Dependency**: Compatible with touch sensors.
- **🗂 Path**: `Scripts/Detection/ObstacleTouchDetector.cs`

---

### 🎮 Simulation and Randomization 🧩

#### **TablePositionRandomizer.cs**
- **✨ Functionality**: Randomizes 🎯 object positions to enhance learning diversity.
- **🎮 Simulation Dependency**: Primarily for virtual environments.
- **🗂 Path**: `Scripts/Simulation/TablePositionRandomizer.cs`

#### **MovementControl.cs**
- **✨ Functionality**: Simulates 🎯 object movements for dynamic task scenarios.
- **🎮 Simulation Dependency**: Designed for simulation-only setups.
- **🗂 Path**: `Scripts/Simulation/MovementControl.cs`

---

### 🧠 Reinforcement Learning 🧩

#### **MoveCubeAgent.cs**
- **✨ Functionality**: Handles agent logic for 🧊 grasping and positioning tasks.
- **🗂 Path**: `Scripts/Agents/MoveCubeAgent.cs`

#### **FinalAgent.cs**
- **✨ Functionality**: Manages advanced multi-task learning, including grasping and placing.
- **🗂 Path**: `Scripts/Agents/FinalAgent.cs`

#### **TouchGrabAgent.cs**
- **✨ Functionality**: Combines touch detection and grasping logic for enhanced 🧠 learning.
- **🗂 Path**: `Scripts/Agents/TouchGrabAgent.cs`

---

## Development 📝

### ⚙️ Hardware Adaptation
1. **📐 Kinematics**: Align 🤖 arm parameters with the hardware kinematic model.
2. **🔧 Sensor Calibration**: Optimize sensor 🧩 for precision.
3. **🤲 Gripper Tuning**: Adjust gripping speed and force based on 🎯 requirements.

### 🌉 Bridging Simulation and Reality
1. Validate 🎮 modules in real-world scenarios.
2. Fine-tune randomized simulations to mimic physical conditions.

### 🛠 Extensibility
- Modular design enables seamless addition of new ✨ and replacement of 🧩.

---

## Unity ML-Agents 🎮 Training Guide

### 🛠 Prerequisites
1. **Unity** (2020.1 or higher).
2. **ML-Agents Toolkit** (Install using `pip install mlagents`).
3. **🐍 Python** (3.6 - 3.9).
4. **🔥 CUDA/cuDNN** (Optional for GPU training).
5. **Configured Unity 🎮 Environment**.

### 📚 Training Steps

#### 1. Modify `FinalAgent.cs`
Update the training condition in `FinalAgent.cs`:

```csharp
if (curriculumDetectValue == 3) {
    // Training logic
}
```

#### 2. 🚀 Start Training
Run the following command:
```bash
mlagents-learn --train Robot_cur3.yaml --run-id=TrainingRun1 --force
```

#### 3. 📊 Monitor Progress
Visualize training 📈 using TensorBoard:
```bash
tensorboard --logdir=results
```

---

## ⚙️ Hardware Operation Workflow

### 📝 Steps
1. **🌐 Network Setup**: Configure `Client.cs` with the system’s 🌐 address.
2. **🔗 Connect to the 🤖**: Ensure successful connection via the program interface.
3. **🎮 Control Process**: Use **▶️ Start**, **⏸ Pause**, and **🔄 Reset** to manage 🤖 arm actions.
4. **✔️ Final Steps**: Verify proper shutdown procedures after operation.

---

This 📦 serves as a comprehensive guide for researchers and developers aiming to implement safe motion planning in industrial 🤖🤝. 
