# 1-26-26 Localization
    Anay Saran, William Hoang

## Goals
- Fix Localization on abruticus for 2026

## Steps
- First step in constants.java, make sure the variable USED_VISION is set to true and then find vision constants.
- Under camera, create an enum variable, give it a name. 
- For the properties you need to go to limelight hardware manager and click find devices. 
- Then take the name and the IP adress, you add :5802.
- In the constructor for nerd drive train, make sure the function set vision is set to true. In the periodic function, in the USED_VISION conditional, create a vision update function for your camera.

## Notes
- When you go to the IP address for the limelights, you can see at the top they are pipelines.
- Pipelines are modes in which the limelights can process vision. In the code, we want disabled to be pipeline 0 and enabled to be pipeline 1.
- However, the actual pipeline is 0 for enabled, becuse the one in the limelight dashboard works for pipeline 0 but not for pipeline 1.
- In the vision update method, we changed the time stamp of the add vision measurement method to use utils.getCurrentTimeSeconds. 
- In the limelights dashboard, in settings we changed the team number to 687. This fixes localization.

## Results
- Vision is added to localization

## Solutions
- Check code and use the right pipeline.