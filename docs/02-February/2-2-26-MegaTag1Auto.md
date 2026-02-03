# 1-16-26 MegaTag1 Auto
    Anay Saran, William Hoang

## Goals
- To make sure MegaTag1 works properly in TeleOp and Auto.

## Notes
- We noticed that when the robot rotates, its position moves, even though it is not supposed to.
- Robot Pose isn't updating on Elastic in auto.
- In Robot.java, there is a function called autonomousInit, there are two methods to reset heading, they are swerveDrive.resetRotation and swerveDrive.zeroFieldOrientation
- We are going to be testing which one we actually need and which one is interfering with our auto.
- and we tried autos but they still went the opposite direction

## Results
- we fixed megatag1 by changing the position of the limelight in the dashboard
- then scheduled a command on teleop init to reset pose with apriltags
 
## Continuation
- hoping to fix that issue or change the forward direction for the robot somehow