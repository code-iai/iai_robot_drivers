# ROS Package for KMS 40 force torque Sensor

To start the driver:
```roslaunch iai_kms_40_driver iai_kms_40_driver.launch```

In order for this to work, your user needs the rights to write to shared memory and set high real-time priorities. 
Using sudo, add the following lines to the file /etc/security/limits.conf and log out and back in:

* YOUR-USER - rtprio 99
* YOUR-USER - memlock 250000
