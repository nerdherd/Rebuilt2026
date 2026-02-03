# Swerve Joystick
    Anthony Ge, Avika Agarwal, Zachary Martinez, Anthony Wheeler

## Goals
- Adjust controller deadband and other filters.
- Simulate drivebase with said filters.

## Filters
- `DeadbandFilter`: If the input is less than the given deadband, return 0. Else, return the input.
- `ScaleFilter`: Multiplies the input by a given scale factor.
- `ClampFilter`: Clamps the input to be between given values.
- `WrapperFilter`: Custom filter. Filter code is passed in.
- `ExponentialSmoothingFilter`: Skipped.

These filters were removed and replaced.

## Results
- Graphed filters in Desmos graphing calculator. [Link.](https://desmos.com/calculator/kdrsdheycv)
- Ran the robot code in simulator and viewed in Elastic.
    - Added `simulationInit` to `Robot.java`
    - Added `simulationPeriodic` to `NerdyDrivetrain.java`
- Moved select translation filters into `translationFilter()` in `SwerveJoystickCommand.java`
- Moved rotation filters into `turningFilter()` in `ServeJoystickCommand.java`
- Changed `kTeleMaxAcceleration` and `kTeleMaxDecelleration` in `Constants.java`
    - Changing these doesn't seem to affect max speed, as it only affects acceleration.

## Continuation
- Test our code with the real drivebase.
