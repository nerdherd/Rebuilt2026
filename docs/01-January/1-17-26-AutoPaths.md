#Make auto paths
    John
    Group: John, Sara

Goals
-
- Make a path for every combination of starting position to non-start positions and non-start to non-start

How to make an Auto Path
-
1. Create a new path
2. Open Waypoints > Start Point > Link Waypoint (Link button)
3. Select the waypoint for the Start Point
4. Repeat steps 2 and 3 for the End Point
5. Scroll down to Ideal Starting State
6. Make sure the rotation is correct by changing the rotation (Deg) value
7. Repeat steps 5 and 6 for Goal End State
8. In the map on the left click on Start Point (Green dot inside of green box) and drag the Control Point (Orange dot connected to Orange Line) onto the End Point (Red dot inside of red box). Repeat with the End point's Control Length [^1]
9. Scroll down on the right side to Path Optimizer and press Optimize, then Accept
10. Check that the robot's path doesn't go over a part of the field that it physically can't go over. If the robot does go over a part of the field drag the Control Points to make the path avoid those points. Try to keep the runtime (The number at the top left of the right window) as small as possible.
11. Name the path according the predetermined naming conventions

New Linked Waypoints
-

|Position Name|X coordinate|Y coordinate|Rotation|   
|-----------|-----------|-----------|-----------| 
|DRF|0.50|5.00|-90|  
|DLF|0.50|7.00|90|

Used for depot intake

- We made some paths and plan to finish the paths by the next meeting
- We strategized autos to make in the future


[^1]: From my testing, making the control length the same distance as the two start points and end points and t hen using the PathPlanner optimizer yields the lowest optimized runtime.