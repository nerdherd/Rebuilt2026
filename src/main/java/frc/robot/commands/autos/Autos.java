package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.commands.PathPlannerAuto;
public final class Autos {
    public static SendableChooser<Command> autonChooserRed = new SendableChooser<>(); //if on red side
    public static SendableChooser<Command> autonChooserBlue = new SendableChooser<>(); //if on blue side

    //autos
    // Example: private static PathPlannerAuto exampleAuto;
    private static Command S3TowerBlue;
    private static PathPlannerAuto S3TowerRed;
    private static PathPlannerAuto S5BottomBlueOutpost;
    private static PathPlannerAuto S4BottomBlueOutpost;
    private static PathPlannerAuto S2TopRedDepot;


    public static void initializeAutos() {
        Shuffleboard.getTab("Autons").add("Red Autons", autonChooserRed)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);
        Shuffleboard.getTab("Autons").add("Blue Autons", autonChooserBlue)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);
        
        //initialize autos
        //Example: exampleAuto = new PathPlannerAuto("example Auto");
        S3TowerBlue = new PathPlannerAuto("S3Blue-Tower");
        S3TowerRed = new PathPlannerAuto("S3Red-Tower");
        S5BottomBlueOutpost = new PathPlannerAuto("S5BottomBlue-Outpost");
        S4BottomBlueOutpost = new PathPlannerAuto("S4BottomBlue-Outpost");
        S2TopRedDepot = new PathPlannerAuto("S2TopRed-Depot");


        //add options to red 
        //autonChooserRed.addOption("exampleRed", exampleAuto);
        autonChooserRed.addOption("S3Tower-Red", S3TowerRed);
        autonChooserRed.addOption("S2TopRed-Depot", S2TopRedDepot);

        //add options to blue
        //autonChooserBlue.addOption("exampleBlue", exampleAuto);
        autonChooserBlue.addOption("S3Tower-Blue", S3TowerBlue);
        autonChooserBlue.addOption("S5BottomBlue-Outpost", S5BottomBlueOutpost);
        autonChooserBlue.addOption("S4BottomBlue-Outpost", S4BottomBlueOutpost);


    }

}
