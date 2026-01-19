package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.subsystems.SuperSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

public class HubDepot extends SequentialCommandGroup{
    public HubDepot(String autoname, SuperSystem superSystem, NerdDrivetrain swerve) throws IOException, ParseException{

        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);
        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();
        
        addCommands(
            // Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            // Commands.runOnce(swerve.getImu()::zeroAll),
            
            Commands.sequence(
                AutoBuilder.followPath(pathGroup.get(0)),
                AutoBuilder.followPath(pathGroup.get(1)),
                AutoBuilder.followPath(pathGroup.get(2)),
            )
        );
    }
}
