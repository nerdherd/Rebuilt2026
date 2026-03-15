package frc.robot.subsystems;

import static frc.robot.Constants.LoggingConstants.kSupersystemTab;
import static frc.robot.Constants.Subsystems.climb;
import static frc.robot.Constants.Subsystems.conveyor;
import static frc.robot.Constants.Subsystems.indexer;
import static frc.robot.Constants.Subsystems.intakeRoller;
import static frc.robot.Constants.Subsystems.intakeSlapdown;
import static frc.robot.Constants.Subsystems.shooter;

import java.util.ArrayList;
import java.util.function.Consumer;

import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.util.NerdyMath;
import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable;

public class SuperSystem implements Reportable {
    public static final ArrayList<TemplateSubsystem> subsystems = new ArrayList<>();
    public NerdDrivetrain swerveDrivetrain;

    public SuperSystem(NerdDrivetrain swerveDrivetrain) {
        this.swerveDrivetrain = swerveDrivetrain;
    }
    
    public static void registerSubsystem(TemplateSubsystem subsystem) {
        subsystems.add(subsystem);
    }
    
    public void applySubsystems(Consumer<TemplateSubsystem> f) {
        for (TemplateSubsystem subsystem : subsystems) f.accept(subsystem);
    }
    
    public Command setShooterCommand(double speed) {
        return Commands.parallel(
            shooter.setDesiredValueCommand(speed)
        );
    }

    // ------------------------------------ subsystems ------------------------------------ //
    public void reConfigureMotors() {
        applySubsystems((s) -> s.applyMotorConfigs());
    }
    
    public void startShoot() {
        indexer.setDesiredValue(7);
        conveyor.setDesiredValue(6);
    }

    public Command shoot() {
        return Commands.runOnce(this::startShoot, indexer, conveyor);
    }

    public Command shootWithCondition() {
        return Commands.run(() -> {
            if (shooter.getCurrentVelocity() > 20.0) {
                startShoot();
                double val = NerdyMath.posMod(MathSharedStore.getTimestamp(), 3.0);
                if (val <= 0.4) intakeRoller.setDesiredValue(-1);
                else if (val <= 1.0) intakeRoller.setDesiredValue(9);
                else intakeRoller.setDesiredValue(0.0);
            } else {
                indexer.setDesiredValue(0);
                conveyor.setDesiredValue(0);
            }
        }, indexer, conveyor)
        .finallyDo(() -> intakeRoller.setDesiredValue(0.0));
    }

    public Command reverseConveyor() {
        return conveyor.setDesiredValueCommand(-4);
    }

    public Command stopConveyor() {
        return conveyor.setDesiredValueCommand(0);
    }
    
    public Command stopShooting() {
        return Commands.parallel(
            indexer.setDesiredValueCommand(0),
            conveyor.setDesiredValueCommand(0)
        );
    }

    public Command spinUpFlywheel() {
        return Commands.parallel(
            setShooterCommand(45)
        );
    }

    public Command spinUpFlywheel(double speed) {
        return Commands.parallel(
            setShooterCommand(speed)
        );
    }
        
    public Command stopFlywheel() {
        return Commands.parallel(
            setShooterCommand(0.0)
        );
    }

    public Command intakeDown() {
        return Commands.sequence(
            intakeSlapdown.setDesiredValueCommand(-8),
            Commands.waitSeconds(0.3),
            intakeSlapdown.setDesiredValueCommand(-2),
            Commands.waitSeconds(0.2),
            intakeSlapdown.setDesiredValueCommand(-1)
        );
    }

    public Command intake() {
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(9)
            );
    }
    
    public Command outtake() {
        return intakeRoller.setDesiredValueCommand(-3);
    }
        
    public Command stopIntaking() {
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(0)
        );
    }

    public Command climbUp() {
        return Commands.sequence(
            climb.setDesiredValueCommand(8),
            Commands.waitSeconds(3.5)
        ).until(() -> climb.getCurrentPosition() > 250.0)
        .andThen(climb.setDesiredValueCommand(0.0));
    }

    public Command climbDown() {
        return Commands.sequence(
            climb.setDesiredValueCommand(-8),
            Commands.waitSeconds(3.5)
        ).until(() -> climb.getCurrentPosition() > 250.0)
        .andThen(climb.setDesiredValueCommand(0.0));
    }

    public Command stopClimb() {
        return climb.setDesiredValueCommand(0);
    }

    public Command shootWithDistance() {
        return Commands.run(
            () -> {
                // calculate distance
                double distance = getHubDistance();
                // convert to rps
                double rps = ShooterConstants.kShootWithDistanceA * distance * distance + ShooterConstants.kShootWithDistanceB;
                // spin up flywheel
                shooter.setDesiredValue(rps);
            }
        );
    }

    public double shootSpeed = 0;
    public DoubleSubscriber shootSpeedSub = null;
    /**
     * change shootSpeed using elastic, always defaults to 0 when 
     * the code is reloaded so save the value
     * @return
     */
    public Command shootWithTuning() {
        if (shootSpeedSub == null) shootSpeedSub = DogLog.tunable(kSupersystemTab + "/Shooter Speed", shootSpeed, (value) -> shootSpeed = value);
        return Commands.run(() -> {
            shooter.setDesiredValue(shootSpeed);
        }, shooter);
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        applySubsystems((s) -> s.setNeutralMode(neutralMode));
    }

    /**
     * fully stops all subsystems by putting them into neutral and disabling them
     * subsystems do not reenable on their own
     * @return a command to stop
     */
    public Command stop() {
        return Commands.runOnce(() -> {
            applySubsystems((s) -> s.stop());
        });
    }   

    public void initialize() {
        applySubsystems((s) -> s.setEnabled(s.useSubsystem));
    }

    public void resetSubsystemValues() {
        applySubsystems((s) -> s.setDesiredValue(s.getDefaultValue()));
        intakeSlapdown.primaryMotor.setPosition(0.0);
    }

    public double getHubDistance() {
        Pose2d hub = FieldPositions.HUB_CENTER.get();
        return swerveDrivetrain.getPose().getTranslation().getDistance(hub.getTranslation());
    }

    // ------------------------------------ logging ------------------------------------ //
    @Override
    public void initializeLogging() {
        applySubsystems((s) -> s.initializeLogging());

        NerdLog.logNumber(kSupersystemTab + "/Hub Distance", this::getHubDistance, "m", LOG_LEVEL.ALL);
        NerdLog.logData(kSupersystemTab + "/Command Scheduler", CommandScheduler.getInstance(), LOG_LEVEL.ALL);
    }
}
