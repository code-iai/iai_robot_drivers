
omni_ethercat2 uses the following parameters:


js_frequency: frequency for the joint_states publisher (Hz)
        example on donbot: 125.0

runstop_frequency: frequency for the hard_run_stop publisher (Hz)
        example on donbot: 10.0

watchdog_period: if a valid command does not arrive in this time, vels=0 (s)
        example on donbot: 0.15

odom_frame_id: name of the odometry frame.
        source: this frame gets set by the localization relative to map
        example on donbot: "/odom"

odom_child_frame_id: name of the child of the odometry frame
        source: this frame has to be in the center of the wheels
        example on donbot: "/base_footprint"

jac_lx: half of distance between front and back axle (m)
        source: measure from CAD
        example on donbot: 0.30375

jac_ly: half of distance centers of left and right wheels (m)
        source: measure from CAD
        example on donbot: 0.39475

drive_constant: encoder ticks for a translation of one meter on a wheel (ticks/m)
        source: specs of encoder, size of wheels
        calculation: 10000 (ticks/turn given by encoder) * gear ratio / ( pi * diameter)
        example on donbot: 10000*20/(3.14159*8*25.4/1000) = 313297.3967 

max_wheel_tick_speed: maximum rotational speed of one motor (ticks/s) 
        source: specs of motors
        calculation: max motor RPM / 60 * (ticks_per_revolution)
        example on donbot: 5000.0 / 60.0 * 10000.0 = 833333.3333

max_dx: x component of maximum allowed twist (m/s)
        source: chose for safety
        example on donbot: 1.0 (m/s)

max_dy: y component of maximum allowed twist (m/s)
        source: chose for safety
        example on donbot: 1.0 (m/s)

max_dtetha: tetha component of maximum allowed twist (rad/s)
        source: chose for safety
        example on donbot: 3.14159 / 4.0 (45deg/s)

odom_x_joint_name: name of first odom joint
        source: according to the URDF
        example on donbot: "odom_x_joint"

odom_y_joint_name: name of second odom joint
        source: according to the URDF
        example on donbot: "odom_y_joint"

odom_z_joint_name: name of the last rotational joint
        source: according to the URDF
        example on donbot: "odom_z_joint"


