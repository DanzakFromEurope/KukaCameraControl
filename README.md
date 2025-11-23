# **🤖 Robot Vision System (KUKA \+ YOLOv8)**

This project integrates a **KUKA Robot** with a **Computer Vision System** (Baumer Camera \+ YOLOv8) to autonomously pick and place objects (cubes).

It uses a "Supervisor-Worker" architecture:

* **Python (Supervisor):** Analyzes images, calculates coordinates, and tells the robot *where* to go.  
* **KUKA (Worker):** Handles the motion planning, safety, and tool activation (Suction/Gripper).

## **📋 1\. System Requirements**

### **Hardware**

* **Robot:** KUKA Robot (tested on KR series) with KRC4 Controller.  
* **Camera:** Baumer Industrial Camera (or Webcam for testing).  
* **Network:** PC and Robot connected via Ethernet.  
  * **PC IP:** 192.168.1.x  
  * **Robot IP:** 192.168.1.152 (Port 7000\)

### **Software (PC)**

* Python 3.9+  
* **Libraries:** opencv-python, ultralytics, py\_openshowvar, pynput, neoapi  
* **YOLO Model:** A trained best.pt model for Oriented Bounding Boxes (OBB).

## **🛠️ 2\. KUKA Controller Setup**

You **MUST** configure the KUKA controller before running the Python script. The Python script relies on specific global variables to communicate.

### **A. Variable Declaration ($config.dat)**

Open R1/System/$config.dat on the robot pendant (SmartPad) and add the following block to the **USER GLOBALS** section:

    ; \---------------------------------------  
    ; PYTHON ROBOT VISION VARIABLES  
    ; \---------------------------------------
    
    ; \-- Manual Jogging Flags \--  
    DECL GLOBAL BOOL goUp=FALSE  
    DECL GLOBAL BOOL goDown=FALSE  
    DECL GLOBAL BOOL goLeft=FALSE  
    DECL GLOBAL BOOL goRight=FALSE  
    DECL GLOBAL BOOL goZUp=FALSE  
    DECL GLOBAL BOOL goZDown=FALSE
    
    ; \-- Tool Control Flags \--  
    DECL GLOBAL BOOL vacuumOn=FALSE  
    DECL GLOBAL BOOL blowOn=FALSE  
    DECL GLOBAL BOOL gripperClose=FALSE
    
    ; \-- Auto Mode Variables \--  
    ; Instead of separate X/Y/R, we use FRAME datatypes for cleaner code  
    DECL GLOBAL FRAME pickPos={X 0.0,Y 0.0,Z 0.0,A 0.0,B 0.0,C 0.0}  
    DECL GLOBAL FRAME dropPos={X 0.0,Y 0.0,Z 0.0,A 0.0,B 0.0,C 0.0}
    
    ; \-- Trigger Flag \--  
    DECL GLOBAL BOOL doPick=FALSE

### **B. Main Program (PythonControl.src)**

Create a new program named PythonControl.src in the R1/Program folder. This program loops infinitely, listening for Python commands.

    DEF PythonControl( )  
         
       ; \--- INITIALIZATION \---  
       INI  
         
       ; Initialize defaults (Important for Velocity)  
       BAS(\#INITMOV,0)  
         
       ; Set Tool and Base (Calibrate these first\!)  
       $TOOL \= TOOL\_DATA\[1\]  
       $BASE \= BASE\_DATA\[1\]  
         
       ; Set Speeds  
       $VEL.CP \= 0.1   ; Manual Jog Speed (0.1 m/s)  
       BAS(\#VEL\_PTP, 20\) ; PTP Speed (20%)  
         
       ; Move to Home  
       PTP {A1 0, A2 \-90, A3 90, A4 0, A5 0, A6 0}  
         
       ; Reset Flags  
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
    
       ; \--- MAIN LOOP \---  
       LOOP  
            
      ; 1\. MAP TOOLS TO OUTPUTS  
      ; CHANGE these numbers \[1\],\[2\] to match your wiring\!  
      $OUT\[1\] \= vacuumOn  
      $OUT\[2\] \= blowOn  
      $OUT\[3\] \= gripperClose

      ; 2\. MANUAL JOGGING (Planar)  
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
        
      ; 3\. MANUAL JOGGING (Height)  
      IF goZUp \== TRUE THEN  
         LIN\_REL {Z 1.0} C\_DIS  
      ENDIF  
      IF goZDown \== TRUE THEN  
         LIN\_REL {Z \-1.0} C\_DIS  
      ENDIF

      ; 4\. AUTO SEQUENCE  
      IF doPick \== TRUE THEN  
           
         ; \--- APPROACH \---  
         pickPos.Z \= 200  
         PTP pickPos  
           
         ; \--- PICK \---  
         pickPos.Z \= 5  ; Pick Height (mm)  
         LIN pickPos  
           
         vacuumOn \= TRUE  
         WAIT SEC 0.5  
           
         ; \--- RETRACT \---  
         pickPos.Z \= 200  
         LIN pickPos  
           
         ; \--- DROP \---  
         dropPos.Z \= 200  
         PTP dropPos  
           
         ; \--- RELEASE \---  
         vacuumOn \= FALSE  
         blowOn \= TRUE  
         WAIT SEC 0.5  
         blowOn \= FALSE  
           
         ; Signal Finished  
         doPick \= FALSE  
           
        ENDIF  
        
       ENDLOOP
    
    END

## **💻 3\. Python Setup & Running**

1. **Install Requirements:**  
   pip install opencv-python numpy ultralytics py\_openshowvar pynput

   *(If using Baumer Camera, install the neoapi wheel manually).*  
2. **Run the Controller:**  
   python main\_robot\_vision.py

### **🎛️ Controls Reference**

| Key | Function |
| :---- | :---- |
| **WASD** | Jog Robot X / Y (Planar Movement) |
| **Q / E** | Jog Robot Z (Height Up / Down) |
| **V** | Toggle Vacuum (Suction) |
| **G** | Toggle Gripper (Open/Close) |
| **B** | Blow Air (Hold key) |
| **X** | **Set Drop-Off Point** (Saves current position as Drop Zone) |
| **C** | **Start Calibration** (Follow on-screen wizard) |
| **ENTER** | **Start/Stop Auto Mode** (Robot picks cubes autonomously) |
| **L** | Toggle Live View (See video while robot moves) |
| **M** | Force Re-calculate Matrix |
| **ESC** | Quit |

## **📐 4\. Calibration Procedure (Must do once\!)**

Before the robot can pick accurately, you must link the Camera Pixels to Robot Millimeters.

1. Place **4 Cubes** on the table (spread them out to the corners).  
2. Press **C** in Python.  
   * The camera will take a snapshot and freeze.  
   * It will label the cubes **\#1, \#2, \#3, \#4**.  
3. **Follow the prompts:**  
   * "Jog to Cube \#1": Use WASD/QE to move the suction cup *directly over* Cube \#1.  
   * Press **C** to confirm.  
   * Repeat for \#2, \#3, \#4.  
4. **Finish:** The system will save calibration\_matrix.npy. You don't need to recalibrate next time unless the camera moves.

## **🐞 Troubleshooting**

* **Robot doesn't move?**  
  * Check ROBOT\_IP in Python.  
  * Ensure **KUKAVARPROXY** is running on the KUKA PC.  
  * Ensure PythonControl.src is **selected and running** (Green play icon) on the pendant.  
* **"Axes don't match array" Error?**  
  * The Baumer camera is sending a weird format. The script attempts to fix this automatically, but verify you are getting Mono8 (Grayscale) images.  
* **Auto Mode does nothing when pressing Enter?**  
  * You must press **X** first to define a Drop-Off location. The robot won't pick if it doesn't know where to put the item.
