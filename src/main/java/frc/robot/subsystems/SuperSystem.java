package frc.robot.subsystems;

import static frc.robot.Constants.Subsystems.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SuperSystem implements Reportable {
    public NerdDrivetrain swerveDrivetrain;

    public SuperSystem(NerdDrivetrain swerveDrivetrain) {
        this.swerveDrivetrain = swerveDrivetrain;
    }
    
    // ------------------------------------ subsystems ------------------------------------ //
    public void reConfigureMotors() {
        intakeSlapdown  .applyMotorConfigs();
        intakeRoller    .applyMotorConfigs();
        conveyor        .applyMotorConfigs();
        indexer         .applyMotorConfigs();
        counterRoller   .applyMotorConfigs();
        shooter         .applyMotorConfigs();
    }
    
    public Command shoot(){
        return Commands.parallel(
            indexer.setDesiredValueCommand(30),
            conveyor.setDesiredValueCommand(25)
        );
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
            shooter.setDesiredValueCommand(50)
        );
    }
        
    public Command stopFlywheel(){
        return Commands.parallel(
            counterRoller.setDesiredValueCommand(0),
            shooter.setDesiredValueCommand(0)
        );
    }

    public Command intakeDown(){
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(0)
        );
    }

    public Command intakeUp(){
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(3.8)
        );
    }

    public Command intakeSlapdownStop() {
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(0.0)
        );
    }

    public Command intake() {
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(35)
        );
    }

    public Command stopIntaking(){
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(0)
        );
    }

    public Command shootWithDistance() {
        return Commands.run(
            () -> {
                // calculate distance
                Pose2d hub = (RobotContainer.IsRedSide()) ? FieldPositions.HUB_CENTER.red : FieldPositions.HUB_CENTER.blue;
                double distance = swerveDrivetrain.getPose().getTranslation().getDistance(hub.getTranslation());
                // convert to rps
                double rps = 0.0018247 * distance * distance + 28.6056; //TODO find conversion equation
                // spin up flywheel
                counterRoller.setDesiredValue(30);
                shooter.setDesiredValue(rps);
            }
        );
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        intakeSlapdown  .setNeutralMode(neutralMode);
        intakeRoller    .setNeutralMode(neutralMode);
        conveyor        .setNeutralMode(neutralMode);
        indexer         .setNeutralMode(neutralMode);
        counterRoller   .setNeutralMode(neutralMode);
        shooter         .setNeutralMode(neutralMode);
    }

    /**
     * fully stops all subsystems by putting them into neutral and disabling them
     * subsystems do not reenable on their own
     * @return a command to stop
     */
    public Command stop() {
        return Commands.runOnce(() -> {
            intakeSlapdown  .stop();
            intakeRoller    .stop();
            conveyor        .stop();
            indexer         .stop();
            counterRoller   .stop();
            shooter         .stop();
        });
    }   

    public void initialize() {
        intakeSlapdown  .setEnabled(useIntakeSlapdown);
        intakeRoller    .setEnabled(useIntakeRoller);
        conveyor        .setEnabled(useConveyor);
        indexer         .setEnabled(useIndexer);
        counterRoller   .setEnabled(useCounterRoller);
        shooter         .setEnabled(useShooter);
        intakeSlapdown.motor1.setPosition(0.0);
    }

    public void resetSubsystemValues() {
        intakeSlapdown  .setDesiredValue(intakeSlapdown.getDefaultValue());
        intakeRoller    .setDesiredValue(intakeRoller.getDefaultValue());
        conveyor        .setDesiredValue(conveyor.getDefaultValue());
        indexer         .setDesiredValue(indexer.getDefaultValue());
        counterRoller   .setDesiredValue(counterRoller.getDefaultValue());
        shooter         .setDesiredValue(shooter.getDefaultValue());
    }

    @Override
    public void initializeLogging() {
        intakeSlapdown  .initializeLogging();
        intakeRoller    .initializeLogging();
        conveyor        .initializeLogging();
        indexer         .initializeLogging();
        counterRoller   .initializeLogging();
        shooter         .initializeLogging();
    }
}
