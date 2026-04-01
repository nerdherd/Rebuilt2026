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
        // autoChooser.addOption("Top-S1NeutralSafe", AutoBuilder.buildAuto("Top-S1NeutralSafe"));
        // autoChooser.addOption("Top-S1NeutralDouble", AutoBuilder.buildAuto("Top-S1NeutralDouble"));
        // autoChooser.addOption("Top-S1NeutralCitrus", AutoBuilder.buildAuto("Top-S1NeutralCitrus"));
        // autoChooser.addOption("Top-S1MidTBDepot", AutoBuilder.buildAuto("Top-S1MidTBDepot"));
        autoChooser.addOption("Top-S1Neutral", AutoBuilder.buildAuto("Top-S1Neutral"));


        // MID
        
        // BOT
        autoChooser.addOption("Bot-S5Neutral", AutoBuilder.buildAuto("Bot-S5Neutral"));
        autoChooser.addOption("Bot-S5NeutralSafe", AutoBuilder.buildAuto("Bot-S5NeutralSafe"));
        // autoChooser.addOption("Bot-S5MidTBOutpost", AutoBuilder.buildAuto("Bot-S5MidTBOutpost"));
        
        NerdLog.logData(kAutosTab + "/Selected Auto", autoChooser, LOG_LEVEL.MINIMAL);
    }

    public static void initNamedCommands(SuperSystem superSystem, NerdDrivetrain swerveDrive) {
        // SWERVE2
        NamedCommands.registerCommand("Reset Pose", swerveDrive.resetPoseWithAprilTags(0.2));

        // INTAKE
        NamedCommands.registerCommand("Intake Down", superSystem.intakeDown());
        NamedCommands.registerCommand("Intake Down Only", superSystem.intakeDownOnly());
        NamedCommands.registerCommand("Intake Hold", superSystem.intakeHold());
        // NamedCommands.registerCommand("Intake Up", superSystem.intakeUp());
        NamedCommands.registerCommand("Intake Start", superSystem.intake());
        NamedCommands.registerCommand("Intake Stop", superSystem.stopIntaking());
        NamedCommands.registerCommand("Intake Down Sequence", 
            Commands.sequence(
                superSystem.intakeDown(),
                superSystem.intake()
            ));
        NamedCommands.registerCommand("Auto Shoot Start", superSystem.startShootWithCondition());
        NamedCommands.registerCommand("Auto Shoot Stop", superSystem.stopShootWithCondition());
        // NamedCommands.registerCommand("Intake Up Sequence", 
        //     Commands.sequence(
        //         superSystem.stopIntaking(), 
        //         superSystem.intakeUp()
        //     ));

        // SHOOTER
        NamedCommands.registerCommand("Flywheel Start", superSystem.spinUpFlywheel());
        NamedCommands.registerCommand("Flywheel Start 0", superSystem.spinUpFlywheel(34));
        NamedCommands.registerCommand("Flywheel Start 45", superSystem.spinUpFlywheel(37));
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
                superSystem.stopShooting(),
                Commands.waitSeconds(1),
                superSystem.stopFlywheel()
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
