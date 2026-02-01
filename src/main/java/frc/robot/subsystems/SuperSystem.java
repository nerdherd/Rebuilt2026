package frc.robot.subsystems;

import static frc.robot.Constants.Subsystems.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SuperSystem implements Reportable {
    public NerdDrivetrain swerveDrivetrain;

    

    //add new subsystems


    public SuperSystem(NerdDrivetrain swerveDrivetrain) {
        this.swerveDrivetrain = swerveDrivetrain;
    }
    
    // ------------------------------------ subsystems ------------------------------------ //
    public void reConfigureMotors() {
        // redo
    }
    
    public Command shoot(){
        return Commands.parallel(
            indexer.setDesiredValueCommand(20)
            );
    }
    
    public Command stopShooting(){
        return Commands.parallel(
            indexer.setDesiredValueCommand(0)
            );
    }

    public Command spinUpFlywheel(){
        return Commands.parallel(
            counterRoller.setDesiredValueCommand(20),
            shooter.setDesiredValueCommand(40)
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
            intakeSlapdown.setDesiredValueCommand(1)
        );
    }

    public Command intakeUp(){
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(-1)
        );
    }

    public Command intakeSlapdownStop() {
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(0.0)
        );
    }

    public Command intake() {
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(10.0)
        );
    }

    public Command stopIntaking(){
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(0)
        );
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        // redo
        reConfigureMotors();
    }

    public Command stop() {
        return Commands.runOnce(() -> {
        // redo
        });
    }   

    public void initialize() {
        //set enabled = true;
        //set initials 
        intakeSlapdown.setEnabled(useIntakeSlapdown);
        intakeRoller.setEnabled(useIntakeRoller);
        indexer.setEnabled(useIndexer);
        counterRoller.setEnabled(useCounterRoller);
        shooter.setEnabled(useShooter);
    }

    @Override
    public void initializeLogging() {
        intakeRoller.initializeLogging();
        indexer.initializeLogging();
        counterRoller.initializeLogging();
        shooter.initializeLogging();
        intakeSlapdown.initializeLogging();
    }

}
