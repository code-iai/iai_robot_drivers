# soft_runstop
soft_runstop is a node that reads a joystick button and turns it
into a software runstop button. It sends a bool message on the topic
/soft_runstop depending on the status.

The software runstop button works as follows: When the button is pressed,
then the robot is stopped. After this event, the button must be released 
for at least 1.5 seconds. Then, another press on the button enables the robot again.
After not receiving a joystick message for at least 0.5 seconds, the robot is
stopped as well.

## Code API

The state machine is implemented in function SoftRunstop::stateMachine().
The timer callback function FILLME acts as a watchdog which stops the
robot when no joystick message was received.
