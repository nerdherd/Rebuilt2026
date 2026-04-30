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

public final class Autos {
    public static SendableChooser<Command> autoChooser = new SendableChooser<>();

    public static void initAutoChooser() {
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption("Test", AutoBuilder.buildAuto("pivot"));

        // EXAMPLE
        // autoChooser.addOption("Auto Name", AutoBuilder.buildAuto("PathPlanner Auto Name"));

        // TOP
        autoChooser.addOption("Top-S1Neutral2.5", AutoBuilder.buildAuto("Top-S1Neutral2.5"));
        autoChooser.addOption("Top-S1Neutral3", AutoBuilder.buildAuto("Top-S1Neutral3"));
        // autoChooser.addOption("Top-S1MidDepot", AutoBuilder.buildAuto("Top-S1MidDepot"));

        // MID
        autoChooser.addOption("Mid-S3DepotTower", AutoBuilder.buildAuto("Mid-S3DepotTower"));
        autoChooser.addOption("Mid-S3DepotTower2", AutoBuilder.buildAuto("Mid-S3DepotTower2"));


        // BOT
        autoChooser.addOption("Bot-S5Neutral2.5", AutoBuilder.buildAuto("Bot-S5Neutral2.5"));

        // TEST
        autoChooser.addOption("Top-S1Trench2.5", AutoBuilder.buildAuto("Top-S1Trench2.5"));
        autoChooser.addOption("Bot-S5Trench2.5", AutoBuilder.buildAuto("Bot-S5Trench2.5"));

        NerdLog.get().logData(kAutosTab + "/Selected Auto", autoChooser, LOG_LEVEL.MINIMAL);
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
        NamedCommands.registerCommand("Intake Hold Stop", superSystem.stopIntakeHold());

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
        NamedCommands.registerCommand("Flywheel Start 45", superSystem.spinUpFlywheel(35.65));
        NamedCommands.registerCommand("Flywheel Start 60", superSystem.spinUpFlywheel(42.9));
        NamedCommands.registerCommand("Flywheel Stop", superSystem.stopFlywheel());
        NamedCommands.registerCommand("Turn to Hub", superSystem.turnToHub(3.0));

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
    
}
