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
        // autoChooser.addOption("Test", AutoBuilder.buildAuto("Test"));

        // EXAMPLE
        // autoChooser.addOption("Auto Name", AutoBuilder.buildAuto("PathPlanner Auto Name"));

        // TOP
        autoChooser.addOption("Top-S1Preload", AutoBuilder.buildAuto("Top-S1Preload"));
        autoChooser.addOption("Top-S2Preload", AutoBuilder.buildAuto("Top-S2Preload"));
        autoChooser.addOption("Top-S1Depot", AutoBuilder.buildAuto("Top-S1Depot"));
        autoChooser.addOption("Top-S1MidT", AutoBuilder.buildAuto("Top-S1MidT"));
        autoChooser.addOption("Top-S1MidTBDepot", AutoBuilder.buildAuto("Top-S1MidTBDepot"));

        // MID
        autoChooser.addOption("Mid-S3PreloadClimb", AutoBuilder.buildAuto("Mid-S3PreloadClimb"));
        autoChooser.addOption("Mid-S3OutpostClimb", AutoBuilder.buildAuto("Mid-S3OutpostClimb"));
        autoChooser.addOption("Mid-S3DepotClimb", AutoBuilder.buildAuto("Mid-S3DepotClimb"));

        // BOT
        autoChooser.addOption("Bot-S4Preload", AutoBuilder.buildAuto("Bot-S4Preload"));
        autoChooser.addOption("Bot-S5Preload", AutoBuilder.buildAuto("Bot-S5Preload"));
        autoChooser.addOption("Bot-S5Outpost", AutoBuilder.buildAuto("Bot-S5Outpost"));
        autoChooser.addOption("Bot-S5MidT", AutoBuilder.buildAuto("Bot-S5MidT"));
        autoChooser.addOption("Bot-S5MidTBOutpost", AutoBuilder.buildAuto("Bot-S5MidTBOutpost"));
        
        NerdLog.logData(kAutosTab + "/Selected Auto", autoChooser, LOG_LEVEL.MINIMAL);
    }

    public static void initNamedCommands(SuperSystem superSystem, NerdDrivetrain swerveDrive) {
        // SWERVE
        NamedCommands.registerCommand("Reset Pose", swerveDrive.resetPoseWithAprilTags(0.2));

        // INTAKE
        NamedCommands.registerCommand("Intake Down", superSystem.intakeDown());
        NamedCommands.registerCommand("Intake Up", superSystem.intakeUp());
        NamedCommands.registerCommand("Intake Start", superSystem.intake());
        NamedCommands.registerCommand("Intake Stop", superSystem.stopIntaking());
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

        // SHOOTER
        NamedCommands.registerCommand("Flywheel Start", superSystem.spinUpFlywheel());
        NamedCommands.registerCommand("Flywheel Start 0", superSystem.spinUpFlywheel(34));
        NamedCommands.registerCommand("Flywheel Start 45", superSystem.spinUpFlywheel(36));
        NamedCommands.registerCommand("Flywheel Start 60", superSystem.spinUpFlywheel(39));
        NamedCommands.registerCommand("Flywheel Stop", superSystem.stopFlywheel());

        NamedCommands.registerCommand("Shoot", superSystem.shoot());
        NamedCommands.registerCommand("Shoot Stop", superSystem.stopShooting());
        NamedCommands.registerCommand("Shoot Distance", superSystem.shootWithDistance());
        NamedCommands.registerCommand("Shoot Ramp Up", 
            Commands.sequence(
                superSystem.spinUpFlywheel(), 
                Commands.waitSeconds(2),
                superSystem.shoot()
            ));
        NamedCommands.registerCommand("Shoot Ramp Down", 
            Commands.sequence(
                superSystem.stopFlywheel(),
                Commands.waitSeconds(1),
                superSystem.stopShooting()
            ));

        // CLIMB
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
    }

}
