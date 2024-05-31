# **ACL_UWB**
# **Aerospace Controls Lab Multi-Agent Ultrawideband (UWB) Relative Pose Estimation Planner and Neural Network**
## Main Objective of Research: 
* Improve UWB pose estimation for multi-agent systems by finding a relationship between real relative pose and UWB sensor range measurements
## Key Steps to Achieve Objective:
* Create a ROS planner that controls an autonomous multi-robot experiment to explore (x,y,yaw, azimuth, elevation) state space exhaustively in order to collect varying motion capture pose data, true distance data, and UWB sensor ranging data.
* Develop a PyTorch fully connected neural network that predicts sensor noise given true pose and distance from autonomous robot data collection.
## High-Level Materials Required to Run Experiment and Complete Reserach:
*  A robotic system featuring a two-wheeled mobile robot capable of xy coordinate and yaw orientation control, augmented by an actuator to manage azimuth and elevation angles for ultrawideband sensors.
*  A motion capture system to collect real pose data
*  Pose estimation algorithms for ultra-wideband sensors
## My High-level Contributions:
* Design a waypoint generator that produces a CSV file containing custom pose data for two robot systems given input parameters, complete with a visualization of both robots' paths.
* Developed robot control pipeline that utilizes a Python-based ROS publisher-subscriber framework to send extended pose data(x,y,yaw,azimuth,elevation) to multiple robot systems, alongside a C++ program that tokenizes (azimuth, elevation) data and operates the actuator.
* Integrate actuator control pipeline into a launch file for efficient and error-free access
* Design and solder wire hardware that allows for easy connection between battery power, Arduino board, and actuator.
* Train, validate, and test fully connected neural network on true pose input data and sensor noise output data to create a model that can accurately predict ultrawideband pose.
## Waypoint Generator Information (waypoint_generator.py)
### Specifications
* The waypoint generator has four types of path styles: stationary, exhaustive, sinusoidal, and circular with each path being customizable given various paramters and pose bounds
* Stationary: The robot's x,y start coordinate is stationary while the yaw, azimuth and elevation are free to change
  <img src="https://github.com/samkrem/ACL_UWB_SLAM/blob/main/images/stationary_path.png" alt="Hello" width="400">

* Exhaustive: The robot systematically explores every possible combination of (x,y,yaw, azimuth elevation) poses within specified ranges and step sizes in a rectangular manner. The direction and starting point of its travel can be adjusted in eight distinct ways, ensuring comprehensive coverage of the entire pose space.
     * snake_1: Starts at bottom left and travels right
     * snake_2: Starts at bottom right and travels left
     * snake_3: Starts at top right and travels left
     * snake_4: Starts at top left and travels right
     * snake_5: Starts at bottom left, travels up
     * snake_6: Starts at bottom right and travels up
     * snake_7: Starts at top right and travels down
     * snake_8: Starts at top left and travels down
<img src="https://github.com/samkrem/ACL_UWB_SLAM/blob/main/images/snake_2.png" alt="Hello" width="400">
* Circular: The robot explores every possible comination of poses in a circular manner given distance between each circle and number of circles.
<img src="https://github.com/samkrem/ACL_UWB_SLAM/blob/main/images/circle_path.png" alt="Hello" width="400">
* Sinusoidal: The robot explores combinations of poses in a sinusoidal manner. The direction of travel (horizontal or vertical), the distance between waves, and the length of each wave can be customized.
<div>
  <img src="https://github.com/samkrem/ACL_UWB_SLAM/blob/main/images/sinusoidal_path.png" alt="Sinusoidal Path 1" style="width: 400px; display: inline-block;">
  <img src="https://github.com/samkrem/ACL_UWB_SLAM/blob/main/images/sinusoidal_path_2.png" alt="Sinusoidal Path 2" style="width: 400px; display: inline-block;">
</div>
* Multiple pose combinations can be combined into one path using the line `np.concatenate(waypoints_list, axis=0`
* In order for a CSV file to be generated, the two robots must have the same total number of poses
* The azimuth angle has range from (-150,150) and the elevation has a range from (-80,80)
### Operation
* Make a CSV filename  (Line 6)
* Go to the generate_waypoints function
* Pick a path for both robots (Line 350) 
* Pick custom parameters and bounds for both robots or choose random, default values (Line 307)
* Combined various paths if need be
* Run generate_waypoints.py
* Now, a CSV file should have been generated
* For more information consult [this slideshow](https://docs.google.com/presentation/d/1mAoERAXJj5MNAZ7o4W73mpC9JDMGl20eGiCZpD5WNRs/edit#slide=id.g2c7d78624b1_2_0)

## Robot Control Pipeline

### Robot System Scheduler (metronome_scheduler_cmd.py)
* This ROS publisher reads the CSV file created by the waypoint generator and sends azimuth and elevation data to the actuator and x,y,yaw data to the wheeled robot
* Pose data is published to two actuators and two wheeled robots through a Float32MultiArray consisting of an individual (x,y,yaw,azimuth,elevation)
* The rate at which data can be sent to the subscriber is customizable.
### Actuator Drivers 1 & 2 (metronome_driver_#.py)
* The metronome driver receives information published from the scheduler and sends it to an Arduino Mega 2560's serial port
* Azimuth and elevation angles are sent through the serial port as a byte string
### Actuator Controller (uwb_platforms_final_python_driver.ino)
* The controller program parses a bytestring into two floats representing an elevation and azimuth angle
* The controller program then controls and operates the actuator using the elevation and azimuth angle
### Wheeled Robot Controller 
* A teammate created this ROS subscriber that takes in information from the Robot System Scheduler and inputs this data into a pure pursuit controller
### Materials and Schematic for Actuator Component of Scheduler
* 1 NUC
* 2x wire harness mounts
* Two AX-12A actuators
* Two 11.1V 3 Cell Lipo Batteries
* Two Arduino Mega 2560s
* Two serial port wires connecting arduino to NUC

   <img src="https://github.com/samkrem/ACL_UWB_SLAM/blob/main/images/Wiring_Diagram.png" alt="Hello" width="800">
### Actuator Wiring and Quickstart Guide 
1. Attach the top servo motor wire to an available connection.
   * The right wire of the top servo connects to mount's ground wire (white or black depending on mount).
   * The free wire that is connected to the top servo motor should go in Mega 2560's TX16 port
<img src="https://github.com/samkrem/ACL_UWB_SLAM/blob/main/images/Step1a.jpg" alt="Hello" width="400"> <img src="https://github.com/samkrem/ACL_UWB_SLAM/blob/main/images/Step1b.jpg" alt="Hello" width="400">
2. Attach the bottom servo motor wire to the other connection.
   * The servos' right wire (it is right wire if 1D robots text is below port) connects to the mount ground wire (white or black depending on mount).
   * The free wire that is connected to the top servo motor should go in Mega 2560's TX18 port
<img src="https://github.com/samkrem/ACL_UWB_SLAM/blob/main/images/Step2a.jpg" alt="Hello" width="400"> <img src="https://github.com/samkrem/ACL_UWB_SLAM/blob/main/images/Step2b.jpg" alt="Hello" width="400">
3. Attach remaining harness ground wire (black or white) into any Arduino ground port
4. Plug blue serial port wire into arduino and nuc
5. Plug LIPO battery into harness mount DEAN connector Repeat 1-4 for each actuator
6. In a terminal: `roscore`
7. In another terminal: type `source ~//metronome_ws/devel/setup.bash In the other terminal:` `roslaunch metronome_controller metronome_actuator.launch`
8. For a google doc version see [this link]([url](https://docs.google.com/document/d/1me-hjQnxL6Q4Z2mmnYES7WfVFdYb4pGamj5aENHr8oY/edit))
## Noise Prediction Model Information (Noise_Prediction_Model.py)
### Neural Network Architecture 
* Input Features: Difference in true pose and distance between robots
* Hidden Layer 1: 256 neurons
* Hidden Layer 2: 128 neurons
* Hidden Layer 3: 64 neurons
* Hidden Layer 4: 32 neurons
* Output Layer: Sensor Noise
* Dropout Rate: 0.1
### Input Specifications
* Pose data ranges: Translational components: (X: 0-3.5m, Y: 0-3.5m, Z: 0-3.5m) and Rotational components: (Roll: -180°-180° , Pitch: -90°-90°, Yaw: -180°-180°). Note: For this experiment specifically, Neural Network input will only be x,y,yaw with complementary azimuth and elevation data although this neural network can be adapted to other UWB pose experiments that estiamte 6D pose.
* True Distance: distance measured between two robots through motion capture data
### Output Specifications
* Mean sensor noise for given distance and relative pose

  




