package frc.robot.subsystems;

import static frc.robot.Constants.LoggingConstants.kSupersystemTab;
import static frc.robot.Constants.Subsystems.*;

import java.util.ArrayList;
import java.util.function.Consumer;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.subsystems.template.TemplateSubsystem;
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
            shooterLeft.setDesiredValueCommand(speed),
            shooterRight.setDesiredValueCommand(speed)
        );
    }

    // ------------------------------------ subsystems ------------------------------------ //
    public void reConfigureMotors() {
        applySubsystems((s) -> s.applyMotorConfigs());
    }
    
    public Command shoot(){
        return Commands.parallel(
            indexer.setDesiredValueCommand(5),
            conveyor.setDesiredValueCommand(4)
        );
    }

    public Command reverseConveyor() {
        return conveyor.setDesiredValueCommand(-4);
    }

    public Command stopConveyor() {
        return conveyor.setDesiredValueCommand(0);
    }
    
    public Command outtake() {
        return intakeRoller.setDesiredValueCommand(-5);
    }
    
    public Command stopShooting(){
        return Commands.parallel(
            indexer.setDesiredValueCommand(0),
            conveyor.setDesiredValueCommand(0)
        );
    }

    public Command spinUpFlywheel(){
        return Commands.parallel(
            counterRoller.setDesiredValueCommand(30),
            setShooterCommand(45)
        );
    }
        
    public Command stopFlywheel(){
        return Commands.parallel(
            counterRoller.setDesiredValueCommand(0),
            setShooterCommand(0.0)
        );
    }

    public Command intakeDown(){
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(-17.433)
        );
    }

    public Command intakeUp(){
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(-11.0)
        );
    }

    public Command intakeSlapdownStop() {
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(0.0)
        );
    }

    public Command intake() {
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(7.5)
        );
    }

    public Command stopIntaking() {
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(0)
        );
    }

    public Command climbUp() {
        return Commands.parallel(
            climb.setDesiredValueCommand(0) // TODO
        );
    }

    public Command climbDown() {
        return Commands.parallel(
            climb.setDesiredValueCommand(0)
        );
    }

    public Command shootWithDistance() {
        return Commands.run(
            () -> {
                // calculate distance
                double distance = getHubDistance();
                // convert to rps
                double rps = 2.9043 * distance * distance + 28.95386; // field day 2/15/2026
                // spin up flywheel
                counterRoller.setDesiredValue(30);
                shooterLeft.setDesiredValue(rps);
                shooterRight.setDesiredValue(rps);
            }
        );
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
        intakeSlapdown.motor1.setPosition(0.0);
    }

    public double getHubDistance() {
        Pose2d hub = FieldPositions.HUB_CENTER.get();
        return swerveDrivetrain.getPose().getTranslation().getDistance(hub.getTranslation());
    }

    

    @Override
    public void initializeLogging() {
        applySubsystems((s) -> s.initializeLogging());

        NerdLog.logNumber(kSupersystemTab + "/Hub Distance", this::getHubDistance, "m", LOG_LEVEL.ALL);
        NerdLog.logData(kSupersystemTab + "/Command Scheduler", CommandScheduler.getInstance(), LOG_LEVEL.ALL);
    }
}
