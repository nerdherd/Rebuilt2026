package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SuperSystem implements Reportable {
    public NerdDrivetrain swerveDrivetrain;
    
    public Intake intake;
    public Conveyor conveyor;
    public Indexer indexer;
    public CounterRoller counterRoller;
    public Shooter shooter;
    public Pivot pivot; 

    //add new subsystems


    public SuperSystem(NerdDrivetrain swerveDrivetrain, Intake intake, Conveyor conveyor, Indexer indexer, CounterRoller counterRoller, Shooter shooter, Pivot pivot) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.intake = intake;
        this.conveyor = conveyor;
        this.indexer = indexer;
        this.counterRoller = counterRoller;
        this.shooter = shooter;
        this.pivot = pivot;

    }
    
    // ------------------------------------ subsystems ------------------------------------ //
    public void reConfigureMotors() {
        // redo
    }
    
    public Command shoot(){
        return Commands.parallel(
            indexer.setEnabledCommand(true),
            indexer.setDesiredValueCommand(20),
            conveyor.setEnabledCommand(true),
            conveyor.setDesiredValueCommand(-3)
            );
    }
    
    public Command stopShooting(){
        return Commands.parallel(
            indexer.setEnabledCommand(false),
            indexer.setDesiredValueCommand(0),
            conveyor.setEnabledCommand(false),
            conveyor.setDesiredValueCommand(0)
            );
    }

    public Command spinUpFlywheel(){
        return Commands.parallel(
            counterRoller.setEnabledCommand(true),
            counterRoller.setDesiredValueCommand(30),
            shooter.setEnabledCommand(true),
            shooter.setDesiredValueCommand(75)
            );
        }
        
    public Command stopFlywheel(){
        return Commands.parallel(
            counterRoller.setEnabledCommand(false),
            counterRoller.setDesiredValueCommand(0),
            shooter.setEnabledCommand(false),
            shooter.setDesiredValueCommand(0)
        );
    }

    public Command intakeDown(){
        return Commands.parallel(
            pivot.setEnabledCommand(true),
            pivot.setDesiredValueCommand(0)
        );
    }

    public Command intakeUp(){
        return Commands.parallel(
            pivot.setEnabledCommand(true),
            pivot.setDesiredValueCommand(6.5)
        );
    }

    public Command pivotStop() {
        return Commands.parallel(
            pivot.setEnabledCommand(false),
            pivot.setDesiredValueCommand(0.0)
        );
    }

    public Command intake() {
        return Commands.parallel(
            intake.setEnabledCommand(true),
            intake.setDesiredValueCommand(4.5)
        );
    }

    public Command stopIntaking(){
        return Commands.parallel(
            intake.setEnabledCommand(false),
            intake.setDesiredValueCommand(0)
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
        // intake.setEnabled(true);
        // indexer.setEnabled(true);
        // counterRoller.setEnabled(true);
        // shooter.setEnabled(true);
        // pivot.setEnabled(true);
        pivot.motor1.setPosition(0.0);
    }

    @Override
    public void initializeLogging() {
        intake.initializeLogging();
        conveyor.initializeLogging();
        indexer.initializeLogging();
        counterRoller.initializeLogging();
        shooter.initializeLogging();
        pivot.initializeLogging();

    }

}
