RoArm-M2-S
​
 요약
​
Overview

RoArm-M2-S is a 4DOF smart robotic arm designed for innovative applications. Adopts lightweight structure design with a total weight of less than 850g and an effective payload of 0.5kg@0.5m, it can be flexibly mounted on various mobile platforms. Adopts a 360°omnidirectional base combined with three flexible joints to create a workspace with a 1-meter diameter.

The joint direct-drive design enhances repositioning precision and also improves structural reliability, with innovative dual-drive technology doubling the shoulder joint torque. The onboard ESP32 MCU main control module supports multiple wireless control modes and provides control interfaces and rich communication protocols for easily connecting to various devices. Compatible with ROS2 and various host computers, supports various wireless and wired communication modes.

Comes with an expansion plate, and supports customizing the EoAT(End of Arm Tooling) to meet innovative application requirements. Provides a user-friendly and cross-platform WEB application that integrates a simple and visualized coordinate control mode, making it easier to get started. Comes with rich graphic and video tutorials to help you learn and use it quickly.

RoArm-M2-S achieves an excellent balance between lightweight, user-friendliness, expandability, and open innovation, it is a multifunctional robotic arm that integrates intelligent control, human-machine interaction, and customizable development. Ideal for applications that require a combination of flexibility, expandability, and user-friendliness.

Features

Expandability: Comes with various expandable components, and supports multifunctional EoAT (End of Arm Toolings) customization to meet different innovative application needs.
Omnidirectional Workspace: The base, with 360° rotation, combined with flexible joint movements, creates a workspace with a diameter of up to 1 meter, enabling versatile movement in all directions.
Easy to use: Offers cross-platform web applications and coordinate control modes to reduce user difficulties, making it easy for users to operate the robotic arm.
Open-source Code: The control code and communication interface documentation for RoArm-M2-S are open-source, facilitating secondary development for users.
Lightweight Structure: Utilizes carbon fiber and 5052 aluminum alloy, resulting in a body weighing less than 850g, making it easy to install on various mobile platforms.
Joint Direct-Drive Design: Uses 12-bit high-precision magnetic encoders to obtain joint angles, ensuring a repeat positioning accuracy of 0.088°, thereby enhancing structural reliability.
Dual-Drive Technology: Innovative dual-drive technology increases the torque of the robotic arm's shoulder joint, improving the overall load capacity of the robotic arm.
Powerful Main Control: Utilizes the ESP32 main control MCU, supporting various control interfaces and wireless communication protocols.
Multi-platform Support: Compatible with ROS2 and various host computers, supporting multiple wireless and wired communication methods, providing flexibility for different development and control requirements.
Rich Tutorials: Offers rich graphic and video tutorials covering various functionalities to help users quickly get started and use the robotic arm, thereby accelerating the realization of innovative applications.
Product Video

RoArm-M2-S video

How to Use

Precautions

Before use, please learn the following:

1. RoArm-M2-S is pre-assembled before shipment, but it is not recommended to disassemble it due to a significant number of servos.
2. The working voltage range of RoArm-M2-S is 7-12.6V. It is recommended to use the 12V 5A power supply, or you can use a 3S lithium battery. It is strictly forbidden to use a power supply that exceeds this working voltage range.
3. It may bring you potential risks that the torque of RoArm-M2-S is large. When using the product, avoid having sensitive areas such as eyes and head within the range of servo movement.
4. Keep this product away from children to prevent injury. And the product can not be subjected to violent impact.
5. For safety reasons, the default demos have a relatively slow speed for the robotic arm's operation. You can refer to subsequent tutorials to change this speed, but excessively low speed may cause shaking when the robotic arm reaches certain positions.
Introduction

RoArm-M2-S Intro.png

The two images above show the markings for various parts of the robotic arm and the commonly used interfaces.
To use, connect the provided 12V 5A power cable to the power interface on the robotic arm, turn on the power switch, and the product's joints will automatically move to the center position.
After powering on, the OLED screen will display the following information:
The first line indicates that the WiFi is in AP mode, and the WiFi hotspot is named RoArm-M2.
The second line indicates that the STA mode is turned off. When WiFi is in STA mode, the router will assign an IP address, which will be displayed.
The content of the third line is the MAC address of the device, which is unique and used for ESP-NOW communication. Please refer to RoArm-M2-S ESP-NOW Control for specific usage.
After powering on, use a smartphone or computer to connect to RoArm-M2-S's WiFi: RoArm-M2, with the password 12345678. Once connected to the WiFi, open the Google Chrome browser and enter 192.168.4.1 in the address bar to access the web-based control interface. From there, you can use the web interface features to control the robotic arm.
Web Usage

Key Function

AngleCtrl: Servo Angle Control

The number displayed above each column of buttons represent the angles to which each joint has rotated, displayed in radians. The labels for each joint can be found on the image in #Introduction. All displayed numbers are updated only after the button is released, and the specific update process is explained below.
RoArm-M2-S-web01.jpg
1. "B L" and "B R" at the first column control the rotation of the base joint, with a rotation range of 360°. When the product is powered on, the base joint will automatically rotate to the middle position, and the number on the button is 0.

Keep clicking on "B L", the base joint turns left at 180°, and the value updates from 0 to 3.14.
Then, keep clicking on "B R", the base joint turns right at 360°, and the value updates from 3.14 to -3.14.
2. "S D" and "S U" at the second column control the rotation of the shoulder joint, with a rotation range of 180°. When the product is powered on, the shoulder joint will automatically rotate to the middle position, and the number on the button is 0.

Keep clicking on "S D", the shoulder joint rotates forward at 90°, and the value updates from 0 to -1.57.
Then, keep clicking on "S U", the shoulder joint rotates reversely at 180°, and the value updates from -1.57 to 1.57.
3. "E D" and "E U" at the third column control the rotation of the elbow joint, with a rotation range of 225°. When the product is powered on, the elbow joint will automatically rotate to the middle position, and the number on the button is 1.57.

Keep clicking on "E D", the elbow joint will rotate downward at 90°, and the value will change from 1.57 to 3.14.
Then keep clicking on "E U", the elbow joint will rotate reversely at 225°, and the value will change from 3.14 to -1.11.
4. "H+ DG" and "H- UG" at the fourth column control the rotation of the end joint. The end joint is divided into "clamp" and "wrist" joints, depending on the end effector's configuration. When the product is powered on, the end joint will automatically rotate to the middle position, and the number on the button is 3.14.

When in "gripper" joint mode, the rotation range is 135°.
Pressing "H+ DG" will make the "gripper" joint grab, which is the default closed state.
Pressing "H- DG" will make the "gripper" joint open, with a maximum opening angle of 135°, and the value will change from 3.14 to 1.08.
When in "wrist" joint mode, the rotation range is 270°.
Keep clicking on "H+ DG", the "wrist" joint will rotate downward at 135°, and the value will change from 3.14 to 5.20.
Then, keep clicking on "H- DG", the "wrist" joint will rotate upward at 270°, and the value will change from 5.20 to 1.08.
INIT: reset all joints to the middle position they were in when the product was powered on.

Torque: Torque Lock Control

Clicking "Torque OFF" means the torque lock is off, then you can manually rotate the joints when the robotic arm is powered on;
Clicking "Torque ON" means the torque lock is on, then you can't manually rotate the joints after the robotic arm is powered on.
Note: if one of the joints or all the joints of the robotic arm receives other rotation commands after the torque lock is turned off, the torque lock will be turned on automatically.

DEFA: Dynamic External Force Adaptation Control

Clicking "DEFA ON" means that the Dynamic External Force Adaptation (DEFA) function is enabled. When DEFA is turned on, if external forces cause the robotic arm to move, it will automatically return to its original position before the external force is applied.
Clicking "DEFA OFF" to disable this function, means that the robotic arm will not automatically return to its original position when external forces are applied.
LED: LED Control

Clicking "LED OFF" to turn off LED lights, and clicking "LED ON" to turn on LEDs.

HORIZONTAL DRAG

Please refer to Horizontal Drag Instructions for more details.

VERTICAL DRAG

Please refer to Vertical Drag Instructions for more details.

COORDCTRL: Robotic Arm End Joint End Point Coordinate Control

X, Y, and Z respectively represent the X axis, Y axis, and Z axis of the "Clamp/Wrist" EoAT (End of Arm Tooling) of the robotic arm. T represents the rotation angle of the EoAT (Different joints have different rotation angles, as you refer to "ANGLECTRL H+DG"), and all of these four parameters are adjusted by "+ -".
INIT: reset all EoAT to the middle position they were in when the product was powered on.
RoArm-M2-S Coord.jpg

FEEDBACK INFORMATION

Input JSON command to communicate with the robotic arm and the feedback of JSON command will be displayed here. The following Below is the commonly used JSON commands shortcut input of the robotic arm. Click the "INPUT" next to the command, it will be automatically input into the input box. For details about the control meaning of the JSON command, see RoArm-M2-S JSON Command Meaning.
RoArm-M2-S-Feedback.jpg

Product Initialization

Users who need to quickly restore their product to the factory program can use the ESP32 download tool for RoArm-M2-S that we provide.

1. Click here to download, unzip it and double-click "flash_download_tool_3.9.5.exe" to open. Then, two windows pop up. The UI interface of the download tool is for operation, and the other window is the terminal to display the working status of the download tool.
2. In the "DOWNLOAD TOOL MODE" interface, select "Chip Type" as ESP32, and "WorkMode" as Factory, and the relative path will be used when calling the binary file, so you don't need to manually enter the binary file path, click OK.
WAVEROVER Demo01.png

3. Enter the "ESP32 FLASH DOWNLOAD TOOL", you can upload the demo for the eight robotic arms at the same time on the right. Connect the driver board on the robotic arm to the PC with a USB cable, and click on "COM" to select the new COM (here is "COM3"). "BAUD" is for setting the download speed, the higher the value, the faster the speed, and ESP32 can use up to 921600.
WAVEROVER Demo07.png

4. After the selection, click on "START" to start uploading the demo, after the upload is completed, "IDLE" will change to "FINISH". Then, the driver board can be disconnected from the USB connection with the computer. Insert the 12V 5A power cable into the 12V DC port of the robotic arm driver board, and then you can control RoArm-M2-S after powering on.
RoArm-M2-S demo02.pngRoArm-M2-S Demo03.png

Onboard Interfaces on General Driver for Robots

700px-General-Driver-for-Robots-details-intro.jpg

No.	Resource Name	Description
1	ESP32-WROOM-32 main control module	Can be developed using Arduino IDE
2	IPEX Gen 1 WIFI interface	Used for connecting the antenna with IPEX1 outer screw inner hole
3	Lidar Interface	Integrated the radar adapter board functionality
4	IIC peripheral expansion interface	Can be used to connect OLED screens or other IIC sensors
5	Reset button	Press and release to reboot the ESP32
6	Download button (BOOT)	When pressed, it boots the ESP32 into download mode
7	DC-DC 5V Regulator circuit	Can power a host device such as a Raspberry Pi or Jetson nano, etc.
8	Type-C port (LADAR)	LiDAR data interface
9	Type-C port (USB)	ESP32 serial communication interface, which can upload programs for ESP32
10	XH2.54 Power connector	Inputs DC7~12.6V, can directly power the serial bus servos and motors
11	INA219	Voltage/current monitoring chip
12	Power switch	Control external power supply ON/OFF (ON position in the figure above)
13	ST3215 Bus Servo Interface	Used to connect ST3215 serial bus servo
14	Motor interface PH2.0 6P	Group B interface for motor with encoder
15	Motor interface PH2.0 6P	Group A interface for motor with encoder
16	Motor interface PH2.0 2P	Group A interface for motor without encoder (LED lamp interface in this product)
17	Motor interface PH2.0 2P	Group B interface for motor without encoder
18	AK09918C	3-axis electronic compass
19	QMI8658	6-axis motion sensor
20	TB6612FNG	Motor control chip
21	Serial bus servo control circuit	Connect multiple ST3215 bus servos and get servo feedback
22	TF card slot	Can be used to store logs or WIFI configurations
23	40PIN expansion interface	Easy access to Raspberry Pi 4B, Raspberry Pi Zero or Jetson Orin Nano
24	40PIN expansion interface	Convenient for using the pins on the host computer installed on the driver board
25	CP2102 chip	Serial port to USB, used for radar data transmission
26	CP2102 chip	Serial to USB, used for ESP32 serial communication
27	Automatic download circuit	Does not require pressing the EN and BOOT buttons when uploading code to ESP32F
RoArm-M2-S Tutorial Directory

RoArm-M2-S ROS2 Humble + Moveit2 Tutorial

M2-S ROS2Humblemian-1.png

RoArm-M2-S ROS2 Humble + Moveit2 Tutorial
RoArm-M2-S User Tutorial

RoArm-M2-S Web Usage
RoArm-M2-S Secondary Development Tool Usage
RoArm-M2-S JSON Command Meaning
RoArm-M2-S WIFI Configuration
RoArm-M2-S Robotic Arm Control
RoArm-M2-S EoAT Setting
RoArm-M2-S FLASH File System Operation
RoArm-M2-S Step Recording and Reproduction
RoArm-M2-S ESP-NOW Control
RoArm-M2-S Python UART Communication
RoArm-M2-S Python HTTP Request Communication
Resource

Robotic Arm Drawing

RoArm-M2-S related drawing
Robotic Arm STEP Model

RoArm-S2-S STEP Model
Open-source Demo

RoArm-M2-S slave example
RoArm-M2-S Python demo
RoArm-M2 series Github open source project address
Software

ESP32 download Tool
Vertical and horizontal control tools
Serial Port Driver

ESP32 CP2102 Serial Port Driver
Driver Board Schematic

Schematic
VirtualBox ROS2 Image

VirtualBox ROS2 Image (System default password: ws)
RoArm-M2-S ROS2 Package

RoArm-M2-S gripper ROS2 package
RoArm-M2-S wrist ROS2 package
FAQ

Question:1. When using serial communication, the robot arm does not respond to JSON commands or ROS2 control?

 Answer:

First, check if the robotic arm is powered on. Refer to the power interface markings in the Product Introduction section for guidance. After plugging in the power interface, ensure that the power switch on the driver board is turned on. Next, check the onboard interfaces part of the General Driver for Robots to ensure that the robot arm is inserted into USB port number ⑨. Finally, verify that the serial port communication's port number is correct. If the control arm does not respond after the above checks, please consult technical support.

{{{5}}}

Question:2. Why does the new COM port not appear when I upload the program to the General Driver for Robots using the download tool or Arduino IDE?

 Answer:

First of all, check whether it is inserted into the USB port of the serial number ⑨ according to the onboard interfaces part of the General Driver for Robots, if yes, then look at the device manager of the computer whether there is a new COM port appearing; if not, check the other devices part if there is an unrecognized device named CP2102 (as shown in the figure).300px-CP2102未识别.png If such a device is found, it's likely due to the lack of a driver. In that case, you can click to install the CP2102 serial port driver. If there is no unrecognized device with CP2102 in the name in other devices part, please contact the store's customer service to return it to the factory for repair.

{{{5}}}

Question:3. When uploading a program to the General Driver for Robots using a download tool or the Arduino IDE, the terminal always shows the "Connecting..." interface, what should I do?

 Answer:

In general, this situation occurs when the automatic download circuit of the General Driver for Robots can not work. At this time, you need to unplug the USB cable to upload again, when "....” appears during unload, press and hold the BOOT button on the General Driver for Robots, then press the EN key for one second and release the EN key, and finally release the BOOT key. If the upload is unsuccessful after multiple attempts, contact the store's customer service to return to the factory for repair.
400px-General Driver for Robots Download.png

{{{5}}}

Question:4. After re-uploading the program to the product, I encounter password errors or other connection issues with Wi-Fi, how to solve the problem?

 Answer:

After re-flashing the program, you need to clear the NVS area with {"T":604} command via serial communication, then power the robotic arm again, and reconnect to the WIFI.

{{{5}}}

Support

Technical Support

If you need technical support or have any feedback/review, please click the Submit Now button to submit a ticket, Our support team will check and reply to you within 1 to 2 working days. Please be patient as we make every effort to help you to resolve the issue.
Working Time: 9 AM - 6 PM GMT+8 (Monday to Friday)

Submit Now




RoArm-M2-S Robotic Arm Control
​
 요약
​
RoArm-M2-S_Web Usage introduces how to control the movement of the robotic arm through the Web app, including the joint angle control and 3D Cartesian coordinates control.
In this tutorial, we will introduce the JSON command meaning for the robotic arm control, including the joint angle control and 3D Cartesian coordinates control. The joint angle control is categorized into control in the form of an angle system and radian system, and the 3D Cartesian coordinates control is for the coordinate position of the endpoint of the robotic arm.
The JSON command shortcuts in this section are located in the "MOVING CTRL" module of the web interface.

Reset

CMD_MOVE_INIT - Move to the Initial Position

{"T":100}
100: indicates this command is CMD_MOVE_INIT, which can rotate all joints of the robotic arm to the initial position.
Under normal circumstances, the robotic arm will automatically move to its initial position when powered on.

This command will cause the process to block.

Joint Angle Control

CMD_SINGLE_JOINT_CTRL - Single Joint Control (in radians)

{"T":101,"joint":0,"rad":0,"spd":0,"acc":10}
101: indicating this command is CMD_SINGLE_JOINT_CTRL, the rotation of a joint of the robot arm is controlled in the form of an arc system.
joint: joint numbers.
1: BASE_JOINT
2: SHOULDER_JOINT
3: ELBOW_JOINT
4: EOAT_JOINT
rad: the angle to be rotated (in radians), taking the initial position of each joint, the default angle and rotation direction of each joint are as follows:
The default initial angle of BASE_JOINT is 0, and its rotation range is 3.14 to -3.14. When the angle increases, the base joint turns left. When the angle decreases, the base joint turns right.
The default initial angle of SHOULDER_JOINT is 0, and its rotation range is 1.57 to -1.57. When the angle increases, the shoulder joint rotates forward. When the angle decreases, the shoulder joint rotates backward.
The default initial angle of ELBOW_JOINT is 1.570796, and its rotation range is 3.14 to -1.11. When the angle increases, the elbow joint rotates downward. When the angle decreases, the elbow joint rotates reversely.
The initial angle of EOAT_JOINT is 3.141593. RoArm-M2-S adopts the clamp by default with a rotation range of 1.08 to 3.14. When the angle decreases, the clamp joint will open. If you adopt the wrist joint, the rotation range is 1.08 to 5.20. When the angle increases, the wrist joint rotates downward. When the angle increases, the wrist joint rotates upward.
spd: The rotation speed, measured in steps per second, is used to control the speed of the servos. In this system, one full rotation of a servo corresponds to 4096 steps. A higher numerical value will result in a faster speed, and when the speed value is set to 0, it will rotate at the maximum speed.
acc: The acceleration at the start and end of rotation can be controlled with a numerical value, which should be a value between 0 and 254, measured in 100 steps per second squared. A smaller numerical value results in smoother acceleration and deceleration. For example, if set to 10, it will accelerate and decelerate at 1000 steps per second squared. When the acceleration value is set to 0, it will use the maximum acceleration.
CMD_JOINTS_RAD_CTRL - All Angle Control (in radians)

{"T":102,"base":0,"shoulder":0,"elbow":1.57,"hand":3.14,"spd":0,"acc":10}
102: indicates the command is CMD_JOINTS_RAD_CTRL, which controls the rotation of all joints for the robotic arm in radians.
base: the angle of the base joints. You can refer to the rotation angle range through the rad key in the "CMD_SINGLE_JOINT_CTRL" command.
shoulder: the angle of shoulder joints
elbow: the angle of elbow joints
hand: the angle of clamp/wrist joints
spd: The speed of rotation, the speed unit is step/second, one revolution of the servo is 4096 steps, the larger the value the faster the speed, when the speed value is 0, it rotates at the maximum speed.
acc: Acceleration at the beginning and end of the rotation, the smaller the value the smoother the start and stop, the value can be 0-254, and the unit is 100 steps per second ^ 2. If the setting is 10, it will be in accordance with the 1000 steps per second of the square of the acceleration and deceleration speed change. When the acceleration value is 0, it will run at the maximum acceleration.
CMD_EOAT_HAND_CTRL - EoAT Control (in radians)

{"T":106,"cmd":3.14,"spd":0,"acc":0}
106: this command is CMD_EOAT_HAND_CTRL for setting the rotation angle of the clamp/wrist joint.
cmd: the rotation angle to be rotated (in radians). the default initial angle of EOAT_JOINT is 3.141593.
RoArm adopts the clamp by default with a rotation range of 1.08 to 3.14. When the angle decreases, the clamp will open.
If you change it to the wrist joint, the rotation range is 1.08 to 5.20. When the angle increases, the wrist joint rotates downward. When the angle decreases, the wrist joint rotates upward.
spd: the speed of rotation, the speed unit is step/second, one revolution of the servo is 4096 steps, the larger the value the faster the speed, when the speed value is 0, it rotates at the maximum speed.
acc: Acceleration at the beginning and end of the rotation, the smaller the value the smoother the start and stop, the value can be 0-254, and the unit is 100 steps per second ^ 2. If the setting is 10, it will be in accordance with the 1000 steps per second of the square of the acceleration and deceleration speed change. When the acceleration value is 0, it will run at the maximum acceleration.
CMD_SINGLE_JOINT_ANGLE - Single Joint Control (in radians)

{"T":121,"joint":1,"angle":0,"spd":10,"acc":10}
121: this command is CMD_SINGLE_JOINT_ANGLE for controlling the rotation of some joints of the robotic arm in radians.
joint: joint number.
1: BASE_JOINT
2: SHOULDER_JOINT
3: ELBOW_JOINT
4: EOAT_JOIN, the angle of the clamp/wrist
angle: the angle to be rotated. Taking the initial position of each joint, the default angle and rotation direction of each joint are as follows:
The default initial angle of BASE_JOINT is 0° with the rotation range of 180° to -180°. When the angle increases, the base joint turns left. When the angle decreases, the base joint turns right.
The default initial angle of SHOULDER_JOINT is 0° with the rotation range of 90° to -90°. When the angle increases, the shoulder joint rotates forward. When the angle decreases, the shoulder joint rotates backward.
The default initial angle of ELBOW_JOINT is 90° with the rotation range of 180° to -45°. When the angle increases, the elbow joint rotates downward. When the angle decreases, the elbow joint rotates upward.
The default initial angle of EOAT_JOINT is 180° with the rotation range of 45°to 180°. When the angle decreases, the clamp will open. If you change it to the wrist joint, the rotation range is 45° to 315°. When the angle increases, the wrist joint rotates downward. When the angle decreases, the wrist joint rotates upward.
spd: The speed of rotation, the speed unit is °/s, the larger the value the faster the speed, when the speed value is 0, it rotates at the maximum speed.
acc: Acceleration at the beginning and end of the rotation, the smaller the value the smoother the start and stop in °/s^2. When the acceleration value is 0, the operation is in accordance with the maximum acceleration.
CMD_JOINTS_ANGLE_CTRL - All Joints Control (In angles)

{"T":122,"b":0,"s":0,"e":90,"h":180,"spd":10,"acc":10}
122: this command is CMD_JOINTS_ANGLE_CTRL controlling the rotation of all joints of the robot arm in angles.
b: the angle of the base joint. For the angle of the base joint, see the description of the angle key in the "CMD_SINGLE_JOINT_ANGLE" command for the range of angular rotation.
s: the angle of the shoulder joint.
e: the angle of the elbow joint.
h: the angle of the clamp/wrist joint.
spd: The speed of rotation, the speed unit is °/s, the larger the value the faster the speed, when the speed value is 0, it rotates at the maximum speed.
acc: Acceleration at the beginning and end of the rotation, the smaller the value the smoother the start and stop, the value can be 0-254 in °/s^2. When the acceleration value is 0, the operation is in accordance with the maximum acceleration.
3D Cartesian Coordinate Control

CMD_SINGLE_AXIS_CRTL - Individual Axis Position Control of Robotic Arm EoAT (inverse kinematics)

{"T":103,"axis":2,"pos":0,"spd":0.25}
The definition of the axes of the robot arm is based on the right-hand rule, with the X-axis positively oriented directly in front of the robot arm, the Y-axis positively oriented to the left of the front of the robot arm, and the Z-axis positively oriented directly above the vertical of the robot arm.

103: Indicates that this command is CMD_SIGNLE_AXIS_CRTL to control the robotic arm motion by giving the coordinate position of a separate axis to the EoAT of the robotic arm.
axis: indicates the axis number. 1-X axis; 2-Y axis; 3-Z axis; 4-T axis, clamp/wrist angle (in radian).
pos: A specific position of an axis in mm. e.g. the example above is to move the EoAT of the robotic arm to position 0 of the Y-axis, which is directly in front of the robotic arm.
spd: The speed of the movement, the larger the value the faster the speed, this move command contains a curve speed control function at the bottom of the command, so the speed is not constant.
This command causes the process to block.

CMD_XYZT_GOAL_CTRL - EoAT Control (inverse kinematics)

{"T":104,"x":235,"y":0,"z":234,"t":3.14,"spd":0.25}
104: indicates this command is CMD_XYZT_GOAL_CTRL for controlling the movement of EoAT.
x, y, z, t: represents the specific position of four axes, the unit is mm. For more details, you can refer to the above introduction of CMD_SINGLE_AXIS_CTRL.
spd: indicates the movement speed. The bigger the value is, the faster the speed is. This movement command includes a curve speed control function at the bottom, so the speed is not constant.
This command may cause the movement to be blocked.

CMD_XYZT_DIRECT_CTRL - EoAT Position Control (inverse kinematics)

{"T":1041,"x":235,"y":0,"z":234,"t":3.14}
1041: Indicates that this instruction is CMD_XYZT_DIRECT_CTRL, which controls the movement of the end point position of the robot arm.
x, y, z, t: The specific position of each of the four axes in mm. For details, refer to the description in the CMD_SINGLE_AXIS_CTRL command above.
Note: the difference between this command and the above command is that this command will not cause blocking. As there is no interpolation calculation in the bottom layer, the robotic arm will move to the target point at the fastest speed after calling this command. It is suitable for the case that a new target point is given continuously by this command, and the difference in the target point position between each command should not be too big.

CMD_SERVO_RAD_FEEDBACK - Get Feedback from Robotic Arm

{"T":105}
105: indicates this command is CMD_SERVO_RAD_FEEDBACK for getting the feedback of the EoAT including coordinates, all joint angles, and the load.
After inputting this command, it feedbacks below:

{"T":1051,"x":309.0444117,"y":3.318604879,"z":238.2448043,"b":0.010737866,"s":-0.004601942,"e":1.570796327,"t":3.141592654,"torB":-56,"torS":-20,"torE":0,"torH":0}
x, y, z: respectively represents the EoAT coordinates of the X, Y, and Z axis.
b, s, e, t: respectively represents the base joint, shoulder joint, elbow joint, and EoAT joint angles (in radian).
torB, torS, torE, torH: respectively represent the load of the base joint, shoulder joint, elbow joint, and EoAT.
CMD_CONSTANT_CTRL - Continuous Movement Control (angle control + inverse kinematics control)

{"T":123,"m":0,"axis":0,"cmd":0,"spd":0}
123: this command is CMD_CONSTANT_CTRL for enabling each joint of the robotic arm or the EoAT of the robotic arm to move continuously after an instruction is input.
m: continuous movement control mode
0: angle control mode
1: coordinate control mode
axis: Control different joint rotations in different modes.
In angle control mode: when the value of m is 0, it controls the angle rotation of all joints for the robotic arm. 1-Base joint, 2-SHOULDER joint, 3-ELBOW joint, and 4-HAND clamp/wrist joint.
Coordinate control mode: when the value of m is 1, it controls the rotation of the EoAT coordinates. 1-X axis, 2-Y axis, 3-Z axis, and 4-HAND clamp/wrist joint.
cmd: movement status.
0-STOP movement.
1-INCREASE: The angle increases in angle control mode; the coordinates increase in inverse kinematics control mode.
2-DECREASE: The angle decreases in angle control mode; the coordinates decrease in inverse kinematics control mode.
spd: the speed coefficient, the bigger the value, the faster the speed. As the rotation speed of each joint is limited, the value is suggested in the range of 0-20.
This chapter introduces the JSON command meaning for the robotic arm, for more learning, you can click here to view.

RoArm-M2-S Tutorial Directory

RoArm-M2-S ROS2 Humble + Moveit2 Tutorial

M2-S ROS2Humblemian-1.png

RoArm-M2-S ROS2 Humble + Moveit2 Tutorial
RoArm-M2-S User Tutorial

RoArm-M2-S Web Usage
RoArm-M2-S Secondary Development Tool Usage
RoArm-M2-S JSON Command Meaning
RoArm-M2-S WIFI Configuration
RoArm-M2-S Robotic Arm Control
RoArm-M2-S EoAT Setting
RoArm-M2-S FLASH File System Operation
RoArm-M2-S Step Recording and Reproduction
RoArm-M2-S ESP-NOW Control
RoArm-M2-S Python UART Communication
RoArm-M2-S Python HTTP Request Communication



Why Use JSON Command to Communicate?
​
 ​요약​
​
​
​
Although we have introduced the basic web-based tutorial for the robotic arm on the main tutorial page, we have designed various JSON format command interfaces to facilitate users in controlling the robotic arm's movements with other devices or programs. In fact, the underlying interface of the web-based control also utilizes JSON commands for communication. The advantages of using JSON format commands to control the robot are as follows:
1. Better Readability:
JSON is a lightweight text data format and is easy for humans to read and compile. With the Key-Value pair format, it is easy to understand and debug, especially for the development and test stage.
2. Easy to Analyze:
As many programming environments adopt JSON analyzer, JSON is easy to analyze making the process of turning the command to the executable operation easier.
3. Cross-platform Compatibility:

JSON is a universal format that can be used on nearly any programming language and platform. This means you can use different programming languages to send and receive JSON commands.
4. Structural Data:
JSON supports nested data structures, including objects and arrays. This allows you to organize commands in a clear manner, including parameters, options, and subcommands.
5. Extensibility:
You can easily add new segments and parameters to JSON commands to support more functions and options without changing the overall structure of the command.
6. Easy to Integrate:
JSON is the standard input and output format for many APP and Web services, which makes the robot seamlessly compatible with other systems and services, for example, communication through REST API.
7. Standard:
JSON is a standard data format with popular support, which means you can operate JSON data with various libraries and tools.
8. Support Multiple Languages:
As JSON can be used in various programming languages, the robot control system programmed in different languages does not need to re-program the command analyzer.
In short, JSON format commands provide a simple, flexible, highly readable, and easily parseable way to control the robot, making the robot control system more robust and maintainable.