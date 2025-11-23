# **Autonomous Robotic Manipulation System with Computer Vision Integration**

This document delineates the technical specifications and operational procedures for a robotic pick-and-place system integrating a KUKA manipulator with a computer vision subsystem (comprising a Baumer industrial camera and the YOLOv8 object detection algorithm).

The system utilizes a hierarchical control architecture, defined as follows:

* **Supervisor (Python):** Performs image acquisition and analysis, computes spatial coordinates, and issues trajectory commands to the robot.  
* **Worker (KUKA Controller):** Executes motion planning, ensures operational safety, and manages end-effector actuation (suction cup and gripper mechanisms).

## **1\. Architectural Component Overview**

The following section details the specific function of each file within the project repository:

### **1.1 main\_robot\_vision.py (Central Control Logic)**

* **Function:** Serves as the primary execution entry point and main control loop.  
* **Responsibilities:**  
  * Establishes and maintains network communication with the KUKA robot and the camera interface.  
  * Captures and processes user input via keyboard peripherals (e.g., WASD teleoperation).  
  * Executes the YOLOv8 inference model on incoming video frames.  
  * Orchestrates the finite state machine governing Manual, Auto, and Calibration modes.  
  * Renders the Graphical User Interface (GUI), including status text, visual indicators, and bounding box overlays.

### **1.2 calibration\_core.py (Coordinate Transformation Engine)**

* **Function:** Manages the mathematical transformation between 2D image space and 3D robot base coordinates.  
* **Key Capabilities:**  
  * add\_point(): Aggregates corresponding pairs of pixel coordinates and robot Cartesian coordinates.  
  * compute\_matrix(): Calculates the Homography Matrix required to map the camera's perspective to the robot's workspace.  
  * save() / load(): Persists the calibration matrix to a .npy file, ensuring system consistency across sessions.  
  * load\_mock\_calibration(): Generates a synthetic matrix to facilitate testing in the Debug/Simulation environment.

### **1.3 camera\_driver.py (Hardware Abstraction Layer)**

* **Function:** Provides a unified interface for image acquisition hardware.  
* **Capabilities:**  
  * Initializes connection to the Baumer Industrial Camera via the neoapi library.  
  * Configures exposure parameters and enables Software Triggering for precise snapshot acquisition.  
  * **Redundancy:** Implements an automatic fallback mechanism to a standard webcam if the industrial camera is undetected.  
  * **Standardization:** Converts image data (e.g., Mono8) into the BGR format required by the OpenCV library.

### **1.4 KUKA\_handler.py (Communication Interface)**

* **Function:** Facilitates network transmission between the PC and the KRC4 controller.  
* **Capabilities:**  
  * Encapsulates the openshowvar library for variable access.  
  * Implements robust read/write operations with defined timeouts (defaulting to 10 seconds) to handle network latency or disconnection.  
  * **Simulation:** Includes a KUKA\_Mock class that emulates robot behavior in memory, allowing for logic verification in the absence of physical hardware.

### **1.5 yolo\_detector.py & vision\_tuner.py (Diagnostic Utilities)**

* **Function:** Standalone scripts for subsystem isolation and testing.  
* **Usage:** These tools allow for the independent verification of camera feeds and AI detection performance, decoupled from the robotic control logic.

## **2\. System Prerequisites**

### **2.1 Hardware Specifications**

* **Manipulator:** KUKA Robot (compatible with KR series) utilizing a KRC4 Controller.  
* **Vision Sensor:** Baumer Industrial Camera (interchangeable with a standard webcam for development purposes).  
* **Network Infrastructure:** Direct Ethernet connection between the workstation and the robot controller.  
  * **Workstation IP:** Subnet 192.168.1.x  
  * **Robot IP:** 192.168.1.152 (Port 7000\)

### **2.2 Software Environment (Workstation)**

* **Runtime:** Python 3.9 or higher.  
* **Dependencies:** opencv-python, ultralytics, py\_openshowvar, pynput, neoapi.  
* **Machine Learning Artifact:** A trained YOLOv8 model (best.pt) optimized for Oriented Bounding Box (OBB) detection.

## **3\. KUKA Controller Configuration**

Configuration of the KUKA controller is a mandatory prerequisite for successful operation. The Python control script depends on the existence of specific global variables within the KUKA system environment.

### **3.1 Variable Declaration ($config.dat)**

The following variable declarations must be appended to the **USER GLOBALS** section of the R1/System/$config.dat file on the robot controller:

    ; \---------------------------------------  
    ; PYTHON ROBOT VISION VARIABLES  
    ; \---------------------------------------
    
    ; \-- Teleoperation Flags \--  
    DECL GLOBAL BOOL goUp=FALSE  
    DECL GLOBAL BOOL goDown=FALSE  
    DECL GLOBAL BOOL goLeft=FALSE  
    DECL GLOBAL BOOL goRight=FALSE  
    DECL GLOBAL BOOL goZUp=FALSE  
    DECL GLOBAL BOOL goZDown=FALSE
    
    ; \-- End-Effector Control Flags \--  
    DECL GLOBAL BOOL vacuumOn=FALSE  
    DECL GLOBAL BOOL blowOn=FALSE  
    DECL GLOBAL BOOL gripperClose=FALSE
    
    ; \-- Autonomous Operation Variables \--  
    ; FRAME datatypes are utilized for consolidated spatial data  
    DECL GLOBAL FRAME pickPos={X 0.0,Y 0.0,Z 0.0,A 0.0,B 0.0,C 0.0}  
    DECL GLOBAL FRAME dropPos={X 0.0,Y 0.0,Z 0.0,A 0.0,B 0.0,C 0.0}
    
    ; \-- Execution Trigger \--  
    DECL GLOBAL BOOL doPick=FALSE

### **3.2 Main Execution Program (PythonControl.src)**

A new program file, designated PythonControl.src, must be created within the R1/Program directory. This program executes a continuous loop to process commands received from the Python supervisor.

    DEF PythonControl( )  
         
       ; Initialize standard motion parameters  
       BAS(\#INITMOV,0)  
         
       ; Initialize Tool and Base Data (Calibration required prior to execution)  
       $TOOL \= TOOL\_DATA\[1\]  
       $BASE \= BASE\_DATA\[1\]  
         
       ; Configure Motion Velocities  
       $VEL.CP \= 0.1     ; Manual Jog Velocity (0.1 m/s)  
       BAS(\#VEL\_PTP, 20\) ; PTP Velocity (20%)  
         
       ; Return to Home Position  
       PTP {A1 0, A2 \-90, A3 90, A4 0, A5 0, A6 0}  
         
       ; Reset Operational Flags  
       goUp=FALSE  
       goDown=FALSE  
       goLeft=FALSE  
       goRight=FALSE  
       goZUp=FALSE  
       goZDown=FALSE  
       doPick=FALSE  
       vacuumOn=FALSE  
       blowOn=FALSE  
       gripperClose=FALSE
    
       ; \--- CONTROL LOOP \---  
       LOOP  
        
      ; 1\. I/O MAPPING  
      ; Note: Output indices \[1\], \[2\], etc., must be mapped to physical wiring  
      $OUT\[1\] \= vacuumOn  
      $OUT\[2\] \= blowOn  
      $OUT\[3\] \= gripperClose

      ; 2\. TELEOPERATION (Planar)  
      IF goUp \== TRUE THEN  
         LIN\_REL {X 1.0} C\_DIS  
      ENDIF  
      IF goDown \== TRUE THEN  
         LIN\_REL {X \-1.0} C\_DIS  
      ENDIF  
      IF goLeft \== TRUE THEN  
         LIN\_REL {Y 1.0} C\_DIS  
      ENDIF  
      IF goRight \== TRUE THEN  
         LIN\_REL {Y \-1.0} C\_DIS  
      ENDIF  
        
      ; 3\. TELEOPERATION (Vertical)  
      IF goZUp \== TRUE THEN  
         LIN\_REL {Z 1.0} C\_DIS  
      ENDIF  
      IF goZDown \== TRUE THEN  
         LIN\_REL {Z \-1.0} C\_DIS  
      ENDIF

      ; 4\. AUTONOMOUS SEQUENCE  
      IF doPick \== TRUE THEN  
           
         ; \--- APPROACH \---  
         pickPos.Z \= 200  
         PTP pickPos  
           
         ; \--- PICK \---  
         ; NOTE: The Z-height may be overridden here if precise mechanical limits require adjustment.  
         ; pickPos.Z \= 5   
         LIN pickPos  
           
         vacuumOn \= TRUE  
         WAIT SEC 0.5  
           
         ; \--- RETRACT \---  
         pickPos.Z \= 200  
         LIN pickPos  
           
         ; \--- DEPOSIT \---  
         dropPos.Z \= 200  
         PTP dropPos  
           
         ; \--- RELEASE \---  
         vacuumOn \= FALSE  
         blowOn \= TRUE  
         WAIT SEC 0.5  
         blowOn \= FALSE  
           
         ; Signal Completion  
         doPick \= FALSE  
           
        ENDIF  
        
       ENDLOOP
    
    END

## **4\. Execution Procedures**

1. Dependency Installation:  
   Execute the following command to install the necessary Python packages:  
   `pip install opencv-python numpy ultralytics py\_openshowvar pynput`

   *(Note: For Baumer camera support, the neoapi wheel must be manually installed).*  
2. System Initialization:  
   Launch the control application:  
   `python main\_robot\_vision.py`

### **Control Interface Reference**

| Input Key | Function |
| :---- | :---- |
| **WASD** | Planar Translation (X/Y Axis) |
| **Q / E** | Vertical Translation (Z Axis) |
| **V** | Toggle Vacuum/Suction Mechanism |
| **G** | Toggle Mechanical Gripper |
| **B** | Activate Air Blow (Hold to activate) |
| **X** | **Define Drop-Off Coordinates** (Records current position) |
| **C** | **Initiate Calibration Routine** (Follow on-screen instructions) |
| **ENTER** | **Toggle Autonomous Mode** (Enables/Disables autonomous picking) |
| **L** | Toggle Live Video Feed (Visual monitoring during actuation) |
| **M** | Force Calibration Matrix Recalculation |
| **ESC** | Terminate Application |

## **5\. Calibration Protocol**

Accurate operation requires a one-time calibration procedure to establish the correlation between the camera's pixel coordinate system and the robot's Cartesian coordinate system.

1. **Preparation:** Position four calibration cubes on the workspace surface, ensuring they are distributed toward the outer corners of the visible area.  
2. **Initiation:** Press **C** within the Python interface.  
   * The system will acquire a static snapshot.  
   * The four detected cubes will be indexed as **\#1, \#2, \#3, and \#4**.  
3. **Data Collection:**  
   * Follow the on-screen prompt: "Jog to Cube \#1". Use the teleoperation keys (WASD/QE) to align the suction cup directly above Cube \#1.  
   * Press **C** to confirm the position.  
   * Repeat this process for cubes \#2, \#3, and \#4.  
4. **Completion:** Upon successful completion, the transformation matrix is computed and serialized to calibration\_matrix.npy. Recalibration is not required for subsequent sessions unless the camera mounting position is altered.

## **6\. Diagnostics and Simulation**

The system architecture supports a comprehensive debug mode, allowing for the testing of logic, file I/O, and calibration routines in the absence of physical robotic hardware.

### **6.1 Enabling Mock Mode**

In main\_robot\_vision.py, modify the configuration as follows:

`DEBUG\_MODE \= True`

* **Virtual Manipulator:** The script instantiates a virtual robot object in memory, initialized at (0, 0, 100).  
* **Visual Feedback:** A blue crosshair indicator is rendered on the camera feed, visualizing the position of the simulated robot relative to the visual field.  
* **Procedure:** Teleoperation keys (WASD) may be used to maneuver the blue crosshair over a target cube, and **C** may be pressed to simulate the calibration data collection process.

### **6.2 Z-Axis (Pick Height) Adjustment**

Should the end-effector attempt to pick from an incorrect elevation (either failing to reach the object or colliding with the surface), the following adjustments are recommended:

Option A: Python Configuration (Recommended)  
Locate the pick\_str formatting line in main\_robot\_vision.py:  
`pick\_str \= "{{X {:.2f}, Y {:.2f}, Z 5.0, ...}}".format(...)`

Modify the Z 5.0 value to the appropriate height in millimeters (e.g., Z 2.0 for a lower approach, Z 10.0 for a higher approach).

Option B: KUKA Controller Override  
Within the PythonControl.src file on the robot controller, uncomment the following line to enforce a hard-coded height limit:  
`; pickPos.Z \= 5`

## **7\. Operational Recommendations**

1. **Illumination:** Consistent and adequate lighting is critical. Shadows may interfere with the OBB (Oriented Bounding Box) angle estimation; dedicated task lighting is recommended.  
2. **Velocity Settings:** It is advisable to commence operations with reduced velocity (`$VEL.CP \= 0.1`). Once system stability is verified, this may be increased (e.g., to 0.5 or 1.0) within the .src file.  
3. **Actuation Latency:** If the manipulator retracts prior to securing the object, the vacuum dwell time (`WAIT SEC 0.5`) should be increased to 0.8 or 1.0 seconds.  
4. **Orientation Ambiguity:** If the manipulator
