package frc.robot.subsystems;

import static frc.robot.Constants.LoggingConstants.kSupersystemTab;
import static frc.robot.Constants.Subsystems.conveyor;
import static frc.robot.Constants.Subsystems.indexer;
import static frc.robot.Constants.Subsystems.intakeRoller;
import static frc.robot.Constants.Subsystems.intakeSlapdown;
import static frc.robot.Constants.Subsystems.leds;
import static frc.robot.Constants.Subsystems.shooter;
import static frc.robot.Constants.Subsystems.useLEDs;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.commands.RebuiltLEDCommand;
import frc.robot.commands.SwerveJoystickCommand;
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
        indexer.setDesiredValue(10);
        conveyor.setDesiredValue(6);
    }

    public Command shoot() {
        return Commands.parallel(
            indexer.setDesiredValueCommand(10),
            conveyor.setDesiredValueCommand(5)
        );
    }

    private double startShootTime = 0.0;
    public Command shootWithCondition() {
        return Commands.run(() -> {
            if (shooter.getCurrentVelocity() > 20.0) {
                if (startShootTime < 0.0) startShootTime = MathSharedStore.getTimestamp();
                startShoot();
                double val = NerdyMath.posMod(MathSharedStore.getTimestamp() - startShootTime, 0.7);
                if (val <= 0.5) intakeRoller.setDesiredValue(-2.0);
                else if (val <= 0.7) intakeRoller.setDesiredValue(9);
                else intakeRoller.setDesiredValue(0.0);
            } else {
                indexer.setDesiredValue(0);
                conveyor.setDesiredValue(0);
            }
        }, indexer, conveyor)
        .finallyDo(
            () -> {
                intakeRoller.setDesiredValue(0.0);
                indexer.setDesiredValue(0);
                conveyor.setDesiredValue(0);
                startShootTime = -1.0;
            }
            );
    }

    public Command autoTurnToHub = null;
    public Command turnToHub(double timeout) {
        if (autoTurnToHub == null) 
            autoTurnToHub = new SwerveJoystickCommand(swerveDrivetrain, () -> 0.0, () -> 0.0, () -> 0.0, () -> true, 
                () -> swerveDrivetrain.angleToPose(FieldPositions.HUB_CENTER) + RobotContainer.kOffset, 
                () -> Translation2d.kZero, () -> false, () -> false, () -> 0.0).finallyDo(() -> swerveDrivetrain.driveRobotOriented(0.0, 0.0, 0.0));

        return autoTurnToHub.raceWith(Commands.waitSeconds(timeout));
    }

    public Command autoShoot = shootWithCondition();
    public Command startShootWithCondition() {
        return Commands.runOnce(() -> CommandScheduler.getInstance().schedule(autoShoot));
    }
    public Command stopShootWithCondition() {
        return Commands.runOnce(() -> CommandScheduler.getInstance().cancel(autoShoot));
    }

    public Command reverseConveyor() {
        return Commands.parallel(
            conveyor.setDesiredValueCommand(-5),
            indexer.setDesiredValueCommand(-5)
            );
    }

    public Command stopConveyor() {
        return Commands.parallel(
            conveyor.setDesiredValueCommand(0),
            indexer.setDesiredValueCommand(0)
            );
    }
    
    public Command stopShooting() {
        return Commands.parallel(
            indexer.setDesiredValueCommand(0),
            conveyor.setDesiredValueCommand(0)
        );
    }

    public Command spinUpFlywheel() {
        return Commands.parallel(
            setShooterCommand(37)
        );
    }

    public Command spinUpFlywheelFeeding() {
        return Commands.parallel(
            setShooterCommand(41)
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
            Commands.waitSeconds(0.2),
            intakeSlapdown.setDesiredValueCommand(-1.5)
        );
    }

    public Command intakeDownOnly() {
        return intakeSlapdown.setDesiredValueCommand(-9);
    }

    public Command intakeHold() {
        return intakeSlapdown.setDesiredValueCommand(-1.5); //change when we have bumpers
    }

    public Command intakeHoldTeleop() {
        return intakeSlapdown.setDesiredValueCommand(-1.5); //change when we have bumpers
    }

    public Command stopIntakeHold() {
        return intakeSlapdown.setDesiredValueCommand(0.0);
    }

    public Command intakeUp() {
        return intakeSlapdown.setDesiredValueCommand(1.5);
    }

    public Command intake() {
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(11),
            intakeHoldTeleop()
            );
    }
    
    public Command outtake() {
        return intakeRoller.setDesiredValueCommand(-8);
    }
        
    public Command stopIntaking() {
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(0),
            stopIntakeHold()
        );
    }

    public Command shootWithDistance(Supplier<Boolean> useBackwards) {
        return Commands.run(
            () -> {
                // calculate distance
                double distance = getHubDistance(useBackwards.get());
                // convert to rps
                double rps = ShooterConstants.kShootWithDistanceA * distance * distance + ShooterConstants.kShootWithDistanceB;
                // spin up flywheel
                shooter.setDesiredValue(Math.min(55.0, rps));
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
        if (shootSpeedSub == null) shootSpeedSub = DogLog.tunable("Shooter Speed", shootSpeed, (value) -> shootSpeed = value);
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
    }

    public double getHubDistance(boolean useBackwards) {
        Pose2d hub = FieldPositions.HUB_CENTER.get();
        return swerveDrivetrain.getLookAheadPose(useBackwards ? ShooterConstants.kBackwardsLookAheadFactor : ShooterConstants.kLookAheadFactor).getTranslation().getDistance(hub.getTranslation());
    }

    public void initializeLEDs() {
        if (!useLEDs) return;
        RebuiltLEDCommand ledCommand = new RebuiltLEDCommand(leds);
        ledCommand.registerIntakeSupplier(() -> intakeRoller.getDesiredValue() > 0.1);
        ledCommand.registerShooterSupplier(() -> (shooter.getDesiredValue() > 0.1) ? NerdyMath.clamp(shooter.getCurrentVelocity() / shooter.getDesiredValue(), 0.0, 1.0) : 0.0);
        ledCommand.registerCountdownSupplier(() -> (DriverStation.getMatchType() != MatchType.None) ? (1.0 - NerdyMath.clamp(RobotContainer.shiftTime / 10.0, 0.0, 1.0)) : 0.0);
        leds.setDefaultCommand(ledCommand);
    }

    // ------------------------------------ logging ------------------------------------ //
    @Override
    public void initializeLogging() {
        applySubsystems((s) -> s.initializeLogging());

        NerdLog.logNumber(kSupersystemTab + "/Hub Distance", () -> getHubDistance(false), "m", LOG_LEVEL.ALL);
        NerdLog.logData(kSupersystemTab + "/Command Scheduler", CommandScheduler.getInstance(), LOG_LEVEL.ALL);
    }
}
