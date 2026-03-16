package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.subsystems.SuperSystem;
import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable.LOG_LEVEL;

import static frc.robot.Constants.LoggingConstants.kAutosTab;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

public final class Autos {
    public static SendableChooser<Command> autoChooser = new SendableChooser<>();

    public static void initAutoChooser() {
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        // autoChooser.addOption("Test", AutoBuilder.buildAuto("Test"));

        // EXAMPLE
        // autoChooser.addOption("Auto Name", AutoBuilder.buildAuto("PathPlanner Auto Name"));

        // TOP
        autoChooser.addOption("Top-S1MidTBDefClimb", AutoBuilder.buildAuto("Top-S1MidTBDefClimb"));
        autoChooser.addOption("Top-S1Neutral", AutoBuilder.buildAuto("Top-S1Neutral"));

        // MID
        autoChooser.addOption("Mid-S3PreloadClimb", AutoBuilder.buildAuto("Mid-S3PreloadClimb"));
        autoChooser.addOption("Mid-S3OutpostClimb", AutoBuilder.buildAuto("Mid-S3OutpostClimb"));
        autoChooser.addOption("Mid-S3DepotClimb", AutoBuilder.buildAuto("Mid-S3DepotClimb"));
        autoChooser.addOption("Mid-S3MidTopBDef", AutoBuilder.buildAuto("Mid-S3MidTopBDef"));
        autoChooser.addOption("Mid-S3MidBotBDef", AutoBuilder.buildAuto("Mid-S3MidBotBDef"));
        

        // BOT
        autoChooser.addOption("Bot-S5MidTBDefClimb", AutoBuilder.buildAuto("Bot-S5MidTBDefClimb"));
        autoChooser.addOption("Bot-S5Neutral", AutoBuilder.buildAuto("BotS5-Neutral"));
        
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
        // NamedCommands.registerCommand("Intake Up Sequence", 
        //     Commands.sequence(
        //         superSystem.stopIntaking(), 
        //         superSystem.intakeUp()
        //     ));

        // SHOOTER
        NamedCommands.registerCommand("Flywheel Start", superSystem.spinUpFlywheel());
        NamedCommands.registerCommand("Flywheel Start 0", superSystem.spinUpFlywheel(34));
        NamedCommands.registerCommand("Flywheel Start 45", superSystem.spinUpFlywheel(34));
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

    public static void initEventMarkers(SuperSystem superSystem, NerdDrivetrain swerveDrive) {
        // INTAKE
        new EventTrigger("Intake Down")
            .onTrue(superSystem.intakeDown());
        new EventTrigger("Intake Down Sequence")
            .onTrue(Commands.sequence(
                superSystem.intakeDown(),
                superSystem.intake()
            ));

        new EventTrigger("Intake Start")
            .onTrue(superSystem.intake());   
        new EventTrigger("Intake Stop")
            .onTrue(superSystem.stopIntaking());

        // FLYWHEEL
        new EventTrigger("Flywheel Start 0")
            .onTrue(superSystem.spinUpFlywheel(34));
        new EventTrigger("Flywheel Start 45")
            .onTrue(superSystem.spinUpFlywheel(33));
        new EventTrigger("Flywheel Start 60")
            .onTrue(superSystem.spinUpFlywheel(39));
        new EventTrigger("Flywheel Stop")
            .onTrue(superSystem.stopFlywheel());

        // SHOOTER
        new EventTrigger("Shoot")
            .onTrue(superSystem.shoot());
        new EventTrigger("Shoot Ramp Down")
            .onTrue(Commands.sequence(
                superSystem.stopShooting(),
                Commands.waitSeconds(1),
                superSystem.stopFlywheel()
            ));
    }

}
