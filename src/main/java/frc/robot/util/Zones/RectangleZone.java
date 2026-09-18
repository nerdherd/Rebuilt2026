package frc.robot.util.zones;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;

public class RectangleZone implements NerdZone {
    public Rectangle2d rect;

    public RectangleZone (Pose2d bottomLeft , Pose2d topRight) {
        this.rect = new Rectangle2d(bottomLeft.getTranslation(), topRight.getTranslation());
    }



    @Override
    public boolean check(Pose2d robotPose) {
        if (this.rect.contains(robotPose.getTranslation())) return true;
        return false;  
    }

    
}
