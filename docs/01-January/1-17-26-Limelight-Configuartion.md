# Limelight Configuration
    James
    Group: James, William

# Notes
- The limelight should be connected to power through VM, 5 volts and 500 milliamps
- The radio should be connected to power though the VRM, 12 Volts 2 amps
- Using ethernet make a connection from the ROBO-RIO to the radio on AUX
- Then from the radio on AUX to the ethernet switch
- Then from the ethernet switch you can connect the Limelights
- Open Limelight hardware manager and click find devices
- Click on the IP address to go to the configuration page
- 

# Adding vision to localization 
- anumeration 
- In constants.java under vision constants for each limelight on the robot you create an anumeration for it using the limelights name and IP address from limelight hardware manager
- To make sure localization is using vision check to see in constants.java that the variable USE_VISION is set to True
- 