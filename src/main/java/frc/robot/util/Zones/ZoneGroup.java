package frc.robot.util.zones;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public class ZoneGroup implements NerdZone {    
    private final List<NerdZone> zones = new ArrayList<NerdZone>();

    public ZoneGroup() {}

    public ZoneGroup(NerdZone... zones) {
        this.zones.addAll(Arrays.asList(zones));
    }

    public ZoneGroup addZone(NerdZone zone) {
        this.zones.add(zone);
        return this;
    }

    @Override
    public boolean check(Pose2d robotPose) {

        for (NerdZone zone : zones) {
            if (zone.check(robotPose)) {
                return true;
            }
        }

        return false;
        
    }
}
