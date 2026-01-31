package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

public final class Autos {

    private static Autos autos;
    public static SendableChooser<Command> autonChooserRed = new SendableChooser<>(); //if on red side
    public static SendableChooser<Command> autonChooserBlue = new SendableChooser<>(); //if on blue side

    //autos
    // Example: private static PathPlannerAuto exampleAuto;
    private static PathPlannerAuto S3TowerBlue;
    private static PathPlannerAuto S3TowerRed;

    public static void initializeAutos() {
        //initialize autos
        //Example: exampleAuto = new PathPlannerAuto("example Auto");
        S3TowerBlue = new PathPlannerAuto("S3Blue-Tower");
        S3TowerRed = new PathPlannerAuto("S3Red-Tower");

        Shuffleboard.getTab("Autons").add("Red Autons", autonChooserRed)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);
        Shuffleboard.getTab("Autons").add("Blue Autons", autonChooserBlue)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);

        //add options to red 
        //autonChooserRed.addOption("exampleRed", exampleAuto);
        autonChooserRed.addOption("S3Tower-Red", S3TowerRed);

        //add options to blue
        //autonChooserBlue.addOption("exampleBlue", exampleAuto);
        autonChooserBlue.addOption("S3Tower-Blue", S3TowerBlue);
    }

}
