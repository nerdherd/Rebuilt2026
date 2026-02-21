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
import com.pathplanner.lib.commands.PathPlannerAuto;
public final class Autos {
    public static SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
    
    public static void initAutoChooser() {
        NerdLog.logData(kAutosTab, "Selected Auto", autoChooser, LOG_LEVEL.MINIMAL);
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
            Commands.waitSeconds(1),
            superSystem.shoot()
        ));

        NamedCommands.registerCommand("Shooter Ramp Down", 
        Commands.sequence(
            superSystem.stopFlywheel(),
            Commands.waitSeconds(1),
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
