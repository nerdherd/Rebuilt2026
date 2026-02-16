package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SuperSystem;

import javax.annotation.processing.SupportedSourceVersion;

import com.pathplanner.lib.auto.NamedCommands;
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
    private static PathPlannerAuto S2TopBlueDepot;


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
        S2TopBlueDepot = new PathPlannerAuto("S2TopBlue-Depot");


        //add options to red 
        //autonChooserRed.addOption("exampleRed", exampleAuto);
        autonChooserRed.addOption("S3Tower-Red", S3TowerRed);
        autonChooserRed.addOption("S2TopRed-Depot", S2TopRedDepot);
        
        //add options to blue
        //autonChooserBlue.addOption("exampleBlue", exampleAuto);
        autonChooserBlue.addOption("S3Tower-Blue", S3TowerBlue);
        autonChooserBlue.addOption("S5BottomBlue-Outpost", S5BottomBlueOutpost);
        autonChooserBlue.addOption("S4BottomBlue-Outpost", S4BottomBlueOutpost);
        autonChooserBlue.addOption("S2TopBlue-Depot", S2TopBlueDepot);
    }

    public static void initializeNamedCommands(SuperSystem superSystem) {
        NamedCommands.registerCommand("Intake Down Sequence", 
        Commands.sequence(
            superSystem.intakeDown(), 
            superSystem.intake()
            ));

        NamedCommands.registerCommand("Intake Up Sequence", 
        Commands.sequence(
            superSystem.stopIntaking(), 
            superSystem.intakeUp()
            ));

        NamedCommands.registerCommand("Shooter Ramp Up", 
        Commands.sequence(
            superSystem.spinUpFlywheel(), 
            Commands.waitSeconds(1),
            superSystem.shoot()
            ));

        NamedCommands.registerCommand("Shooter Ramp Down", 
        Commands.sequence(
            superSystem.stopFlywheel(), 
            superSystem.stopShooting()
            ));

        NamedCommands.registerCommand("Wait", Commands.waitSeconds(1));
        NamedCommands.registerCommand("SpinUpFlywheel", superSystem.spinUpFlywheel());
        NamedCommands.registerCommand("StopFlywheel", superSystem.stopFlywheel());
        NamedCommands.registerCommand("Shoot", superSystem.shoot());
        NamedCommands.registerCommand("StopShooting", superSystem.stopShooting());
        NamedCommands.registerCommand("IntakeUp", superSystem.intakeUp());
        NamedCommands.registerCommand("IntakeDown", superSystem.intakeDown());
        NamedCommands.registerCommand("Intake", superSystem.intake());
        NamedCommands.registerCommand("StopIntaking", superSystem.stopIntaking());

    }

}
