# ROS package for Schunk WSG-50 Gripper
Forked from: [https://github.com/nalt/wsg50-ros-pkg](https://github.com/nalt/wsg50-ros-pkg)
Which was originally forked from: [https://code.google.com/p/wsg50-ros-pkg](https://code.google.com/p/wsg50-ros-pkg)

Modifications of this repository:
Reading back state with high rates, open-loop control via topics, catkinized, modifications for hydro.
Existing features are not discussed here - see original Wiki: [https://code.google.com/p/wsg50-ros-pkg/wiki/wsg_50](https://code.google.com/p/wsg50-ros-pkg/wiki/wsg_50)


## Node wsg\_50\_ip (was: wsg\_50_tcp)

### Parameters
* *ip*: IP address of gripper
* *port*: Port of gripper
* *local_port*: Local port for UDP
* *protocol*: udp or tcp (default)
* *rate*: Polling rate in Hz.
* *grasping_force*: Set grasping force limit on startup


### Services
Only *set_acceleration* and *set_acknowledge* from [https://code.google.com/p/wsg50-ros-pkg/wiki/wsg_50](https://code.google.com/p/wsg50-ros-pkg/wiki/wsg_50). Services currently block the reception of state updates.

### Topics
* *~/goal\_position [IN, wsg_50_msgs/PositionCmd]:*<br/>
Position goal; send target position in mm, speed and force
* *~/goal\_speed [IN, wsg_50_msgs/PositionCmd]:*<br/>
Velocity goal (in mm/s) and force; positive velocity values open the gripper
* *~/moving [OUT, std_msgs/Bool]*:<br/>
Signals a change in the motion state for position control. Can be used to wait for the end of a gripper movement. Does not work correctly yet for velocity control, since the gripper state register does not directly provide this information.
* *~/state [OUT, std_msgs/State]:*<br/>
State information (opening width, speed, forces). Note: Not all fields are available with all communication modes.
* */joint_states [OUT, sensor_msgs/JointState]:*<br/>
Standard joint state message



### Control
Allows for closed-loop control with a custom script (see below) that supports up to 2 FMF finger. Gripper state is read synchronously with the specified rate. Up to 30 Hz could be reached with the WSG-50 hardware revision 2. The gripper can be controlled asynchronously by sending position or velocity goals to the topics listed above. Commands will be sent with the next read command in the timer callback timer_cb().<br />
The service interface can still be used - yet, they are blocking the gripper communication.

### Gripper script
The script *cmd_measure.lua* must be running on the gripper for the script mode. It allows for non-blocking position and velocity control and responds with the current position, speed, motor force and up to two FMF finger forces. The custom commands 0xB0 (read only), 0xB1 (read, goal position and speed), 0xB2 (read, goal speed) are used. Tested with firmware version 2.6.4. There have been minor API changes since 1.x.<br />
To automatically run the script on startup of the gripper, open the gripper webpage and use the *File Manager* under *Scripting* to upload the file *cmd_measure.lua* to the gripper. Now select "Enable Startup Script" under Settings->System->StartupScript. In the following popup select the script you just uploaded.
