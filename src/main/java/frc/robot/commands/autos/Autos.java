package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SuperSystem;
import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable.LOG_LEVEL;

import static frc.robot.Constants.LoggingConstants.kAutosTab;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


public final class Autos {
    public static SendableChooser<Command> autoChooser = new SendableChooser<>();

    public static void initAutoChooser() {
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        // autoChooser.addOption("S3Bottom-Neutral", AutoBuilder.buildAuto("S3Bottom-Neutral"));
        // autoChooser.addOption("S4Bottom-Neutral", AutoBuilder.buildAuto("S4Bottom-Neutral"));
        // autoChooser.addOption("S5Bottom-Neutral", AutoBuilder.buildAuto("S5Bottom-Neutral"));

        // autoChooser.addOption("S1Top-Neutral", AutoBuilder.buildAuto("S1Top-Neutral"));
        // autoChooser.addOption("S2Top-Neutral", AutoBuilder.buildAuto("S2Top-Neutral"));
        // autoChooser.addOption("S3Top-Neutral", AutoBuilder.buildAuto("S3Top-Neutral"));
        autoChooser.addOption("S1Top-Depot", AutoBuilder.buildAuto("S1Top-Depot"));
        // autoChooser.addOption("S2Top-Depot", AutoBuilder.buildAuto("S2Top-Depot"));
        autoChooser.addOption("S3Preload", AutoBuilder.buildAuto("S3Preload"));
        
        // autoChooser.addOption("S4Bottom-Outpost", AutoBuilder.buildAuto("S4Bottom-Outpost"));
        autoChooser.addOption("S5Bottom-Outpost", AutoBuilder.buildAuto("S5Bottom-Outpost"));
        autoChooser.addOption("S5Bottom-OutpostNeutral", AutoBuilder.buildAuto("S5Bottom-OutpostNeutral"));

        
        NerdLog.logData(kAutosTab + "/Selected Auto", autoChooser, LOG_LEVEL.MINIMAL);
    }

    public static void initNamedCommands(SuperSystem superSystem) {
        NamedCommands.registerCommand("Intake Down Sequence", 
        Commands.parallel(
            superSystem.intakeDown(), 
            superSystem.intake()
        ));

        NamedCommands.registerCommand("Intake Up Sequence", 
        Commands.parallel(
            superSystem.stopIntaking(), 
            superSystem.intakeUp()
        ));

        NamedCommands.registerCommand("Shooter Ramp Up", 
        Commands.sequence(
            superSystem.spinUpFlywheel(), 
            Commands.waitSeconds(2),
            superSystem.shoot()
        ));

        NamedCommands.registerCommand("Shooter Ramp Down", 
        Commands.sequence(
            superSystem.stopFlywheel(),
            Commands.waitSeconds(1),
            superSystem.stopShooting()
        ));

        NamedCommands.registerCommand("Climb Up", 
        Commands.sequence(
            superSystem.climbUp(),
            Commands.waitSeconds(11.0)
        ));

        NamedCommands.registerCommand("Climb Down", 
        Commands.sequence(
             superSystem.climbDown(),
            Commands.waitSeconds(9.0),
            superSystem.stopClimb()
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
