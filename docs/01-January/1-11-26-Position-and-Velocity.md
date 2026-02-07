# Postion and Velocity Template Subsystem Test
    Daniel

- We made a new file called Flywheel
- We extended Flywheel from Template Subsystem
- We added motor configurations
- We added variables for desired values and if the robot is enabled
- We added subsystem mode which is an enum to choose whether we are using position or velocity
- We added a follower request using this template
`public TemplateSubsystem(String name, int motor1ID, int motor2ID, boolean reverseMotor2, SubsystemMode mode, double defaultValue)`
- Then we programmed which motor would be the follower
- We added the mode and value that we wanted and gave the robot the velocity or position voltage
- We ran the robot with elastic to look at the velocity and position of the robot
- While testing we noticed a problem with the postion of the pivot and are currently trying to fix it
- The problem is that it's zero position is in the high negatives
- The pivot postion was fixed by adding the gear ratio back
- There was a problem with the rollers due to using the constant class which was fixed by reverting back to the flywheel class
- **Fixed flywheel position control with motion magic setting cruise velocity and acceleration**

---


# Configuring Radio
    Rohan

- Important details
    - The 1st led (led on the left) signifies if power is being transmitted to the radio.
    - The 2nd led signifies the status of the radio.
    - The 3rd and 4th leds signify if the radio is connected to the wifi.
    - The 5th led (led on the right) signifies if the radio is connected to the roborio.
- Steps to Configure:
1. Connect ethernet from the computer to the radio
2. Connect ethernet from the radio to the roborio
3. Connect 1 pair of power cables from the roborio to the PDH
4. Connect another pair of power cables from the PDH to the VRM.
5. Connect the last pair of power cables from the VRM to the radio.
6. Connect the battery to the PDH and turn it on.
7. Download the VividHosting software from the VividHosting website.
8. Open the software and run the decrypt check zone to upload to the radio
