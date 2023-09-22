# **ACL_UWB**
##**Directions:**
    - Follow instructions for setting up electronics.
    - Verify connections and that correct Arduino code is uploaded
    - Set up turtlebot nucs and a command station. 
    - Create a roscore.
    - Source metronome_controller package (~/workspace/uwb-workspace/metronome_ws/devel/setup.bash).
    - Run metronome drivers (rosrun metronome_controller MetronomeDriver#.py) for each turtlebot (they must have different node names for ideal operation) 
    - Run metronome scheduler on command station (rosrun metronome_controller MetronomeScheduler.py) 
 
## **UNDERSTANDING MetronomeScheduler.py CODE: **
### Description: The "master" for the UWB metronome experiment. Plans elevation angles and turtlebot distance based on simulation of two drones at varying heights and distances. Sends the planned data to the turtlebot and servo motor drivers. Refer to image below for reference.
    - ** \_\_init()\_\_** Initializes variables target_angle_pub (target angle publisher), and bot_dist (turtlebot distance but too be changed in future version)
    - **angle_distance_planner()**: Plans for elevation angles (degrees) and turtlebot distance using (x,z) coordinates, sends elevation angles and distances through angle\_sender and distance\_sender funsctions.
    - **angle\_sender(ang)**: Publishes target_angle as Int16, sleeps to allow for servo to move
    - **distance\_sender(ang)**: Publishes dist as Int16, sleeps to allow for turtlebot to move
    - **round(num)**: rounds a float angle to an integer
![ACL_UWB](https://github.com/fishberg/urop-metronome/blob/main/images/Metronome_Transformation_Diagram.png)

## **UNDERSTANDING MetronomeDriver1.py CODE: **
### Description: Subscribes to target_angle topic and sends that info through the serial port to the AX12 servo motor. 
### Functions:
    - **\_\_init()\_\_**: Initializes variables arduino_port (the serial port connection) and target_angle_sub (subscribes to target angle topic)
    - **target_angle_callback (msg: Int16)**: Subscriber callback converts angle Int16 into 8 bit byte equivilant, writes it to arduino, and arduino logs response


 ## **UNDERSTANDING uwb_platforms_final_python_driver.ino CODE:** 
 ### Description: Arduino script that directly controls upper servo motor through python driver angle inputs
 ### **Important variables**: 
    - #define BaudRate (1000000): Way of configuring baud rate for AX12A.h servo motors **(Note: if baudrate isn't 1000000, none of the code will work)** 
    - curr\_servo\_angle: current angle for upper servo motor
    - prev\_servo\_angle: previous angle for upper servo motor
 ###Important functions: 
    - move\_to\_angle(ang, sp): moves upper servo motor 

## **UNDERSTANDING uwb_platforms_final_user_input.ino CODE**
### Description: Arduino script that directly controls upper and lower servo motors through serial user input
### **Important variables**:
    - #define BaudRate (1000000): Way of configuring baud rate for AX12A.h servo motors **(Note: if baudrate isn't 1000000, none of the code will work)**
    - yawstep: the yaw angle in degrees to turn each step (originally set to 15) 
    - pitchstep: the pitch angle in degrees to turn after each complete yaw sweep (originally set to 15) 
    - timestep: the time in seconds it waits between changing angles 
    - min_pitch_angle: the lowest angle in degrees the pitch can get with respect to the vertical direction (recommended setting: -90) 
    - max_pitch_angle: the highest angle in degrees the pitch can get with respect to the vertical direction (recommended setting: 90) 
    - min_yaw_angle: the complete sweep range of the yaw angle in degrees (recommended setting -150)
    - max_yaw_angle: the complete sweep range of the yaw angle in degrees (recommended setting 150)
### **IMPORTANT FUNCTIONS IN CODE (see code for other annotations if needed):**
    - custom_sequence(int sec, int ang1, int ang2): adjusts lower and upper servos by a specified amount of degrees within the max_yaw_angle and max_pitch_angle bounds
    - testing_sequence(int sec): adjusts lower and upper servos motor at many different orientations 
    - move_to_angleself(int num, int ang, int sp, int t): moves servo to desired location, where inputting a number (1 for lower servo, 2 for upper servo) will turn desired servo
    - subscriberCallback: ROS subscriber callback method that returns an elevation angle

## **INSTRUCTIONS FOR SETTING UP THE ELECTRONICS:**
1. Check that the connections of the Arduino MEGA are consistent with the image below 

        1.1 Black wire goes to ground
        1.2 Purple wire (from lower servo) goes to TX1 (Communications Port 18) 
        2.4 Green wire (from upper servo) goes to TX2 (Communications Port 16)

![ACL_UWB](https://github.com/fishberg/urop-metronome/blob/main/images/Servo_Wiring_1.png)

![ACL_UWB](https://github.com/fishberg/urop-metronome/blob/main/images/Servo_Wiring_2.png)

2. Before running the program and connecting all the wiresconnect the MM dean to the Lipo/Power Supply 
        
        2.1 Turn on the power supply, and set its voltage to **11.1 volts** (coarse voltage adjusts voltage quicker whereas fine voltage adjusts voltage more precisely)
        2.2 Use a 11.1V 3 cell Lithium Polymer (Lipo) battery (DEAN connector)
![ACL_UWB](https://github.com/fishberg/urop-metronome/blob/main/images/Servo_Wiring_3.png)

3. If using power supply, turn it on
4. To start over, click the reset button on the Arduino. To stop, the easiest way is to turn off the power



