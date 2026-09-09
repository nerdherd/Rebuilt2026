package frc.robot.util.Zones;

import edu.wpi.first.math.geometry.Pose2d;

public class RectangleZone implements NerdZone {

    public Pose2d bottomLeft, topRight;

    public RectangleZone (Pose2d bottomLeft , Pose2d topRight) {
        this.bottomLeft = bottomLeft;
        this.topRight = topRight;
    }



    @Override
    public boolean check(Pose2d robotPose) {
        if (robotPose.getX() >= bottomLeft.getX() && robotPose.getX() <= topRight.getX() && robotPose.getY() >= bottomLeft.getY() && robotPose.getY() >= topRight.getY()) {
            return true;
        }
        return false;  
    }

    
}
