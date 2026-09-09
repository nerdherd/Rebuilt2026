package frc.robot.util.Zones;

import edu.wpi.first.math.geometry.Pose2d;

public class CircleZone implements NerdZone {
    
    public final Pose2d center;

    public final double radius;

    public CircleZone (Pose2d center, int radius) {
        this.center = center; 
        this.radius = radius;
    }

    @Override
    public boolean check(Pose2d robotPose) {
        double dx = center.getX() - robotPose.getX();
        double dy = center.getY() - robotPose.getY();

        return dx*dx+dy*dy <= radius*radius;
    }
}
