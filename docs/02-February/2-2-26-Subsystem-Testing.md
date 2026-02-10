# Subsystem Testing
    John
    Group: Mason, Ayush, Aadi

Goal
-
Test the “More Efficient Mockup” subsystems

Results
-
- Problem: outake driver motor makes loud noise when stopping after running
  - Solution 1: Change Kp values (Failed)
  - Root of issue: Old code disables subsystems after each run, new code does not
  - Solution 2: Disable subsystem after each run by merging main into "More Efficient Mockup" (Success)

- Glazed Joel


