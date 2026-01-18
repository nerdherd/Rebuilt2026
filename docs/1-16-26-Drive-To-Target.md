# 1-16-26 Drive To Target
    Mason Tsai, Anay Saran, Anthony Wheeler

## Goals
- Test Drive To Target on Abruticus for functionality
- Add RingDriveCommand

## Results
- Licensed Abruticus motors.
- Found issue with odometry; need to set operator heading correctly.
- Learned about calculating angles using atan2.
- Added RingDriveCommand and related constants in Constants.

## Continuation
- Test RingDriveCommand.

## Notes
- Thread Safety: parallel processes may interact and attempt to change or read values at the same time. Objects should be locked to prevent this.
- Use atan2 to find the closest point on a circle to a given point.