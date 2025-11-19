# KukaCameraControl

## USER GLOBALS on KUKA Controller
    ; ---------------------------------------
    ; PYTHON ROBOT VISION VARIABLES
    ; ---------------------------------------
    
    ; -- Manual Jogging Flags --
    DECL GLOBAL BOOL goUp=FALSE
    DECL GLOBAL BOOL goDown=FALSE
    DECL GLOBAL BOOL goLeft=FALSE
    DECL GLOBAL BOOL goRight=FALSE
    
    ; -- Tool Control Flags --
    DECL GLOBAL BOOL vacuumOn=FALSE
    DECL GLOBAL BOOL blowOn=FALSE
    DECL GLOBAL BOOL gripperClose=FALSE
    
    ; -- Auto Mode Variables --
    DECL GLOBAL REAL target_x=0.0
    DECL GLOBAL REAL target_y=0.0
    DECL GLOBAL REAL target_r=0.0
    DECL GLOBAL REAL drop_x=0.0
    DECL GLOBAL REAL drop_y=0.0
    
    ; -- Trigger Flag --
    DECL GLOBAL BOOL doPick=FALSE



## KUKA Control file (must be running)
    DEF PythonControl( )
       
       ; --- INITIALIZATION ---
       INI
       
       ; Set your Tool and Base (Calibrate these first!)
       $TOOL = TOOL_DATA[1]
       $BASE = BASE_DATA[1]
       
       ; Move to a safe home position
       PTP {A1 0, A2 -90, A3 90, A4 0, A5 0, A6 0}
       
       ; Reset all flags
       goUp = FALSE
       goDown = FALSE
       goLeft = FALSE
       goRight = FALSE
       doPick = FALSE
       vacuumOn = FALSE
       blowOn = FALSE
       gripperClose = FALSE
    
       ; --- MAIN LOOP ---
       LOOP
          
          ; 1. MAP VARIABLES TO PHYSICAL OUTPUTS
          ; Change the numbers [1], [2]... to match your wiring
          $OUT[1] = vacuumOn
          $OUT[2] = blowOn
          $OUT[3] = gripperClose
    
          ; 2. MANUAL JOGGING
          ; Moves in small increments while Python holds the flag TRUE
          IF goUp == TRUE THEN
             LIN_REL {X 1.0} C_DIS
          ENDIF
          
          IF goDown == TRUE THEN
             LIN_REL {X -1.0} C_DIS
          ENDIF
          
          IF goLeft == TRUE THEN
             LIN_REL {Y 1.0} C_DIS
          ENDIF
          
          IF goRight == TRUE THEN
             LIN_REL {Y -1.0} C_DIS
          ENDIF
    
          ; 3. AUTO PICK & PLACE SEQUENCE
          IF doPick == TRUE THEN
             
             ; A. Move up to Safe Z (Avoid hitting table)
             LIN {Z 200}
             
             ; B. Move to Target (X, Y from Python)
             ; Note: We use A (Rotation) from Python for the Gripper Angle
             PTP {X target_x, Y target_y, Z 200, A target_r, B 0, C 180}
             
             ; C. Move Down to Pick Height (e.g. Z=10mm)
             LIN {Z 10}
             
             ; D. Activate Suction
             vacuumOn = TRUE
             WAIT SEC 0.5
             
             ; E. Move Up
             LIN {Z 200}
             
             ; F. Move to Drop Off Location
             PTP {X drop_x, Y drop_y, Z 200, A 0, B 0, C 180}
             
             ; G. Release
             vacuumOn = FALSE
             blowOn = TRUE
             WAIT SEC 0.5
             blowOn = FALSE
             
             ; H. Signal Python we are done
             doPick = FALSE
             
          ENDIF
          
       ENDLOOP
    
    END
