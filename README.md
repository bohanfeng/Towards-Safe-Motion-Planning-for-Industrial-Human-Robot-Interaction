# Towards-Safe-Motion-Planning-for-Industrial-Human-Robot-Interaction
Code Repository for the Paper: "Towards Safe Motion Planning for Industrial Human-Robot Interaction"

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

