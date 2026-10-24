package frc.robot.subsystems;

import static frc.robot.Constants.LoggingConstants.kSupersystemTab;
import static frc.robot.Constants.Subsystems.conveyor;
import static frc.robot.Constants.Subsystems.indexer;
import static frc.robot.Constants.Subsystems.intakeRoller;
import static frc.robot.Constants.Subsystems.intakeSlapdown;
import static frc.robot.Constants.Subsystems.leds;
import static frc.robot.Constants.Subsystems.shooter;
import static frc.robot.Constants.Subsystems.useLEDs;
import static frc.robot.Constants.Subsystems.hood;

import java.util.ArrayList;
import java.util.function.Consumer;
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
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ZoneConstants;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.commands.RebuiltLEDCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.subsystems.template.TemplateSubsystem.SubsystemMode;
import frc.robot.util.NerdyMath;
import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable;

public class SuperSystem implements Reportable {
    
    public static final ArrayList<TemplateSubsystem> subsystems = new ArrayList<>();

    // ----------------------------------++ DRIVE ++----------------------------------- //
    public NerdDrivetrain swerveDrivetrain;
    
    public SuperSystem(NerdDrivetrain swerveDrivetrain) {
        this.swerveDrivetrain = swerveDrivetrain;
    }
    
    public Command autoTurnToHub = null;
    public Command turnToHub(double timeout) {
        return autoTurnToHub = new SwerveJoystickCommand(swerveDrivetrain, () -> 0.0, () -> 0.0, () -> 0.0, () -> true, 
                () -> swerveDrivetrain.angleToPose(FieldPositions.HUB_CENTER) + RobotContainer.kOffset, 
                () -> Translation2d.kZero, () -> false, () -> false, () -> 0.0).finallyDo(() -> swerveDrivetrain.driveRobotOriented(0.0, 0.0, 0.0))
                    .raceWith(Commands.waitSeconds(timeout));

    }

    // ---------------------------------++ SUBSYSTEMS ++--------------------------------- //
    // ------------------------------------- intake ------------------------------------- //
    public Command intake() {
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(11),
            intakeHoldTeleop()
            );
    }
    
    public Command outtake() {
        return intakeRoller.setDesiredValueCommand(-9.5);
    }
        
    public Command stopIntaking() {
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(0),
            stopIntakeSlapdown()
        );
    }

    // --------------------------------- intake pivot --------------------------------- //
    public Command intakeDownAuto() {
        return Commands.sequence(
            intakeSlapdown.setDesiredValueCommand(-8),
            Commands.waitSeconds(0.2),
            intakeSlapdown.setDesiredValueCommand(-1.5)
        );
    }

    public Command intakeDownOnlyAuto() {
        return intakeSlapdown.setDesiredValueCommand(-10);
    }

    public Command intakeHoldAuto() {
        return intakeSlapdown.setDesiredValueCommand(-1.5); //change when we have bumpers
    }

    public Command intakeHoldTeleop() {
        return intakeSlapdown.setDesiredValueCommand(-1); //change when we have bumpers
    }

    public Command stopIntakeSlapdown() {
        return intakeSlapdown.setDesiredValueCommand(0.0);
    }

    public Command intakeUp() {
        return intakeSlapdown.setDesiredValueCommand(1.5);
    }

    // ------------------------------------ conveyor ------------------------------------ //
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

    public void startShoot() {
        indexer.setDesiredValue(10);
        conveyor.setDesiredValue(8);
        switch(shooterState) {
            case SHOOTING:
                if (useHoodShoot()) setHood(0.5);
                else setHood(0.0);
                break;
            case PASSING:
                if (ZoneConstants.kLongPass.get().check(swerveDrivetrain.getPose())) {
                    setHood(1.0);
                    hood.positionController.FeedForward = 2.0;
                } else {
                    setHood(1.0);
                    hood.positionController.FeedForward = 0.0;
                }
                break;
            case NONE:
            default: setHood(0.0); break;
        }
    }

    public Command stopShoot() {
        return Commands.parallel(
            indexer.setDesiredValueCommand(0),
            conveyor.setDesiredValueCommand(0),
            hoodDownCommand(),
            Commands.runOnce(() -> hood.positionController.FeedForward = 0.0)
        );
    }

    public Command startShootCommand() {
        return Commands.runOnce(() -> startShoot());
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

    // ------------------------------------ shooter ------------------------------------ //
    public Command autoShoot = shootWithCondition();
    
    public Command startShootWithCondition() {
        return Commands.runOnce(() -> CommandScheduler.getInstance().schedule(autoShoot));
    }
    public Command stopShootWithCondition() {
        return Commands.runOnce(() -> CommandScheduler.getInstance().cancel(autoShoot));
    }

    public Command autoShootWithDistance = autoShootWithDistance().finallyDo(() -> {shooter.setDesiredValue(0.0);});
    
    public Command startShootWithDistance() {
        return Commands.runOnce(() -> CommandScheduler.getInstance().schedule(autoShootWithDistance));
    }

    public Command stopShootWithDistance() {
        return Commands.runOnce(() -> CommandScheduler.getInstance().cancel(autoShootWithDistance));
    }
    
    public Command autoShootWithDistance() {
        return Commands.run(
            () -> {
                // calculate distance
                double distance = getHubDistance();
                double rps = 0.0;
                // convert to rps
                rps = ShooterConstants.kShootWithDistanceA * distance * distance + ShooterConstants.kShootWithDistanceB;
                // spin up flywheel
                shooter.setDesiredValue(Math.min(55.0, rps));
            }, shooter);
    }

    public Command shootWithDistance() {
        return Commands.run(
            () -> {
                shooterState = ShooterState.SHOOTING;
                // calculate distance
                double distance = getHubDistance();
                double rps = 0.0;
                if (useHoodShoot()) {
                    // convert to rps
                    rps = ShooterConstants.kShootWithDistanceHoodA * distance * distance + ShooterConstants.kShootWithDistanceHoodB;
                } else {
                    // convert to rps
                    rps = ShooterConstants.kShootWithDistanceA * distance * distance + ShooterConstants.kShootWithDistanceB;
                }
                // spin up flywheel
                shooter.setDesiredValue(Math.min(55.0, Math.max(0.0, rps)));
            }, shooter);
    }

    public double shootSpeed = 0;
    public DoubleSubscriber shootSpeedSub = null;
    /**
     * change shootSpeed using elastic, always defaults to 0 when 
     * the code is reloaded so save the value
     * @return
     */
    public Command flywheelWithTuning() {
        if (shootSpeedSub == null) shootSpeedSub = DogLog.tunable("Shooter Speed", shootSpeed, (value) -> shootSpeed = value);
        return Commands.run(() -> {
            shooterState = ShooterState.SHOOTING;
            shooter.setDesiredValue(shootSpeed);
        }, shooter);
    }
    
    public Command setFlywheelCommand(double speed) {
        return Commands.either(
            shooter.setDesiredValueCommand(speed),
            stopFlywheel(),
            () -> speed != 0.0);
    }

    // ------------------------------------ flywheel ------------------------------------ //
    public Command startFeeding() {
        return Commands.run(() -> {
            shooterState = ShooterState.PASSING;
            if (ZoneConstants.kLongPass.get().check(swerveDrivetrain.getPose())) 
                shooter.setDesiredValue(55);
            else 
                shooter.setDesiredValue(45);
        });
    }
        
    public Command stopFlywheel() {
        return Commands.parallel(
            shooter.setDesiredValueCommand(0.0),
            setShooterStateCommand(ShooterState.NONE)
        );
    }

    // ------------------------------------ hood ------------------------------------ //
    /** set the shooter's hood's position
     * @param value between 1 and 0, where 1 is up and 0 is down
    */
    public void setHood(double value) {
        value = NerdyMath.clamp(value, 0.0, 1.0);
        hood.setDesiredValue((HoodConstants.kUpPos-HoodConstants.kDownPos) * value + HoodConstants.kDownPos);
    }

    public Command setHoodCommand(double value) {
        return Commands.runOnce(() -> setHood(value));
    }
    
    public Command hoodDownCommand() {
        return setHoodCommand(0.0);
    }

    public Command hoodUpCommand() {
        return setHoodCommand(1.0);
    }

    // ------------------------------------ helper functions ------------------------------------ //
    public enum ShooterState{NONE, SHOOTING, PASSING};
    public ShooterState shooterState = ShooterState.NONE;
    public Command setShooterStateCommand(ShooterState state) {
        return Commands.runOnce(() -> shooterState = state);
    }

    public boolean useHoodShoot() {
        return !Constants.ZoneConstants.kShootingGroup.check(swerveDrivetrain.getPose()); 
    }

    public double getHubDistance() {
        Pose2d hub = FieldPositions.HUB_CENTER.get();
        return swerveDrivetrain.getLookAheadPose(ShooterConstants.kLookAheadFactor).getTranslation().getDistance(hub.getTranslation());
    }

    public void initializeLEDs() {
        if (!useLEDs) return;
        RebuiltLEDCommand ledCommand = new RebuiltLEDCommand(leds);
        ledCommand.registerIntakeSupplier(() -> intakeRoller.getDesiredValue() > 0.1);
        ledCommand.registerShooterSupplier(() -> (shooter.getDesiredValue() > 0.1) ? NerdyMath.clamp(shooter.getCurrentVelocity() / shooter.getDesiredValue(), 0.0, 1.0) : 0.0);
        ledCommand.registerCountdownSupplier(() -> (DriverStation.getMatchType() != MatchType.None) ? (1.0 - NerdyMath.clamp(RobotContainer.shiftTime / 10.0, 0.0, 1.0)) : 0.0);
        leds.setDefaultCommand(ledCommand);
    }


    // ------------------------------------ supersystem functions ------------------------------------ //
    public static void registerSubsystem(TemplateSubsystem subsystem) {
        subsystems.add(subsystem);
    }
    
    public void applySubsystems(Consumer<TemplateSubsystem> f) {
        for (TemplateSubsystem subsystem : subsystems) f.accept(subsystem);
    }
    
    public void reConfigureMotors() {
        applySubsystems((s) -> s.applyMotorConfigs());
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        applySubsystems((s) -> s.setNeutralMode(neutralMode));
    }
    
    /**
     * fully stops all subsystems by putting them into neutral and disabling them
     * subsystems do not reenable on their own
     * @return a command to stop
     */
    public void stop() {
        applySubsystems((s) -> s.stop());
    }

    public void enableSubsystems() {
        applySubsystems((s) -> s.setEnabled(s.useSubsystem));
    }

    public void resetSubsystemValues() {
        applySubsystems((s) -> {
            s.setDesiredValue(s.getDefaultValue());
            if (s.mode == SubsystemMode.POSITION)
                s.primaryMotor.setPosition(s.getDefaultValue());
        });
    }

    // ------------------------------------ logging ------------------------------------ //
    @Override
    public void initializeLogging() {
        applySubsystems((s) -> s.initializeLogging());

        NerdLog.getNT().logNumber(kSupersystemTab + "/Hub Distance", () -> getHubDistance(), "m", LOG_LEVEL.MEDIUM);
        NerdLog.get().logData(kSupersystemTab + "/Command Scheduler", CommandScheduler.getInstance(), LOG_LEVEL.ALL);
        NerdLog.getNT().logBoolean(kSupersystemTab + "/useShootHood", () -> useHoodShoot() , LOG_LEVEL.MEDIUM);
    }
}
