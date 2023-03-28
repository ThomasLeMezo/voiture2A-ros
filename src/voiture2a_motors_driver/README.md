# Voiture2a_motors_driver

## Description
This driver is used to control the engine and servo of the Voiture2a robot.

## Interfaces
The node subscribe to a geometry_msgs/Twist message where:
- linear.x control the engine.
- angular.z control the servo.

Note that values are clamped between -1.0 and 1.0.

The node publish a MotorsState message that contains:
- the values of the pwm send to the engine and servo.
- the values of the remote control channels, failsafe and lost signal
- the battery voltage
It also publishes the value send to the dspic inside a Engine.msg message
that is used for log.

## Safety
The node automatically send a stop command to servo and engine if no messages 
are received for more than 1.0s by default.

## Parameters
- loop_dt [long, 100]: loop period in ms
- delay_stop [int, 1000]: safety delay with no data in ms
- reverse_servo [bool, false]: reverse servo direction
- reverse_engine [bool, false]: reverse engine direction