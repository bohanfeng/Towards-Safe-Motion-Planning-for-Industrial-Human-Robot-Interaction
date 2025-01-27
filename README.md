# Towards-Safe-Motion-Planning-for-Industrial-Human-Robot-Interaction
Code Repository for the Paper: "Towards Safe Motion Planning for Industrial Human-Robot Interaction"

# Unity ML-Agents Training Guide

## Overview

This guide provides instructions for training an agent using Unity ML-Agents Toolkit, including necessary code modifications and steps to run the training session.

## Prerequisites

Before starting, ensure the following:

1. **Unity** (version 2020.1 or higher recommended).
2. **ML-Agents Toolkit**:
   - Install the Python package:
     ```bash
     pip install mlagents
     ```
3. **Python** (3.6 - 3.9 recommended).
4. **CUDA and cuDNN** (if training with GPU).
5. **Unity Environment**: Ensure your Unity project is correctly configured with the ML-Agents package and your agent is properly set up.

## Steps to Train the Agent

### 1. Modify `FinalAgent.cs`

1. Locate the `FinalAgent.cs` file in your Unity project ( in the `Scripts` folder).
2. Open the file in your preferred code editor.
3. Navigate to **line 482**. Change the condition `== 0` to `== 3`. For example:

   **Before:**
   ```csharp
   if (curriculumDetectValue == 0)  //parameter selection 3 for training 0 for running
   {
       // Existing code
   }
   ```

   **After:**
   ```csharp
   if (curriculumDetectValue == 3)  //parameter selection 3 for training 0 for running
   {
       // Existing code
   }
   ```

4. Save the file and return to Unity.

### 2. Run the Training Command

Execute the following command in your terminal or command prompt to start training:

```bash
mlagents-learn --train Robot_cur3.yaml --run-id=XXXXXX --force
```

#### Command Explanation

- `--train`: Enables training mode.
- `Robot_cur3.yaml`: Specifies the training configuration file.
- `--run-id=XXXXX`: Sets a unique identifier for this training session.
- `--force`: Overwrites existing data for the same run ID.

### 3. Play the Unity Environment

1. Open the Unity Editor.
2. Ensure the Unity environment is set up and configured for ML-Agents training.
3. Click the **Play** button in Unity to start the environment.

   - Unity will now interact with the ML-Agents Python API.
   - Training progress will be logged in the terminal and saved in the results folder.

### 4. Monitor Progress

- Use **TensorBoard** to visualize training metrics:
  ```bash
  tensorboard --logdir=results
  ```
  Open `http://localhost:6006` in your browser to view progress.

### 5. Training Output

Once training is complete:
1. The trained model files (e.g., `.onnx`) will be saved in the `results/XXXX` directory.
2. These models can be loaded back into Unity for evaluation or inference.

## Notes

- Ensure the Unity environment is active and running when executing the `mlagents-learn` command.
- Changing the condition in `FinalAgent.cs` might impact the agent's behavior. Make sure this change aligns with your desired training setup.
- If encountering issues, check Unity's Console for errors or Python logs for troubleshooting.

## Troubleshooting

- Verify the `Robot_cur3.yaml` file syntax and parameters.
- Ensure Unity is connected to the Python API during training.
- For detailed logs, use the `--debug` flag:
  ```bash
  mlagents-learn --train Robot_cur3.yaml --run-id=version2test1 --force --debug
  ```


# Hardware Operation Workflow Guide

## Prerequisites
- Ensure that the robotic arm device is connected and powered on.
- Verify that the computer is properly connected to the network.

## Step-by-Step Instructions

### 1. Check Network Configuration
Before starting the robotic arm, confirm the computer's network configuration:
- Open Command Prompt (Windows) or Terminal (Linux/macOS) and execute the following command:
  ```bash
  ipconfig
  ```
- Locate the local IP address of your computer and note it down for later use in the `ClientReader`.

### 2. Modify IP Address
- Open the `ClientReader` folder and locate the `Client.cs` file.
- Edit the file as follows:
  - Navigate to line 83.
  - Replace the IP address on this line with the IP address recorded in Step 1.
    ```csharp
    // Replace with your local IP address
    string serverIp = "192.168.X.X";  // Example: 192.168.1.100
    ```

### 3. Run the Program
- After editing the `Client.cs` file, save and close it.
- Perform the following steps:
  - Open and run the `ClientReader` program.
  - Click the **Run** button in the program interface to start the client connection.

### 4. Connect to the Server
- Click the **Connection** button.
- Check the console output to ensure that it displays **"Connection Successful."**

### 5. Arm Operation Process
After completing the **Connection** step, follow this sequence to operate the robotic arm:
- Click the **Reset** button to ensure the robotic arm is in its initial state.
- Click the **Start** button to activate the robotic arm.

### 6. Pause and Resume
During the operation, if pausing is required:
- Click the **Pause** button to halt the robotic arm.
- After pausing, perform these steps:
  - Click the **End** button to finish the current action.
  - Click the **Reset** button to reset the robotic arm.
  - Click the **Start** button to restart the robotic arm.

### 7. Complete the Operation
- Repeat steps 5 and 6 until all actions are executed.
- Ensure all operations are finalized before shutting down the robotic arm or the program.

