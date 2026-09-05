package frc.robot.util.Zones;

import edu.wpi.first.math.geometry.Pose2d;

public class SemicircleZone implements NerdZone{
    
    public Pose2d center;
    public double radius;


    public SemicircleZone (Pose2d center, double radius) {
        this.center = center;
        this.radius = radius; 
    }


    @Override
    public boolean check(Pose2d robotPose) {

        if (Math.hypot((center.getX() - robotPose.getX()), (center.getY()- robotPose.getY())) > radius) {
            return false;
        }
        //dot
        return ((center.getX() - robotPose.getX()) * center.getRotation().getCos() + (center.getY()- robotPose.getY()) * center.getRotation().getSin()) > 0;

    }

    





}
