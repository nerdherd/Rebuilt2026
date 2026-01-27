package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SuperSystem implements Reportable {
    public NerdDrivetrain swerveDrivetrain;
    
    public Intake intake;
    public Indexer indexer;
    public CounterRoller counterRoller;
    public Shooter leftShooter;
    public Shooter rightShooter;
    public Pivot pivot; 

    //add new subsystems

    public enum ExecutionOrder {
        //TODO
    }

    public SuperSystem(NerdDrivetrain swerveDrivetrain,Intake intake, Indexer indexer, CounterRoller counterRoller, Shooter leftShooter, Shooter rightShooter, Pivot pivot) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.intake = intake;
        this.indexer = indexer;
        this.counterRoller = counterRoller;
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        this.pivot = pivot;

    }
    
    // ------------------------------------ subsystems ------------------------------------ //
    public void reConfigureMotors() {
        // redo
    }
    public Command shoot(){
        return Commands.parallel(
            counterRoller.setDesiredValueCommand(20),
            indexer.setDesiredValueCommand(20),
            leftShooter.setDesiredValueCommand(30),
            rightShooter.setDesiredValueCommand(30)
            );
    }
    
    public Command stopShooting(){
        return Commands.parallel(
            counterRoller.setDesiredValueCommand(0),
            indexer.setDesiredValueCommand(0)
            );
    }

    public Command spinUpFlywheel(){
        return Commands.parallel(
            leftShooter.setDesiredValueCommand(20),
            rightShooter.setDesiredValueCommand(20)
        );
    }

    public Command stopFlywheel(){
        return Commands.parallel(
            leftShooter.setDesiredValueCommand(0),
            rightShooter.setDesiredValueCommand(0)
        );
    }

    public Command intake(){
        return Commands.sequence(
            pivot.setDesiredValueCommand(.12),
            intake.setDesiredValueCommand(10)
        );
    
    }
    public Command stopIntaking(){
        return Commands.sequence(
            intake.setDesiredValueCommand(0),
            pivot.setDesiredValueCommand(0)
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
        intake.setEnabled(true);
        indexer.setEnabled(true);
        counterRoller.setEnabled(true);
        leftShooter.setEnabled(true);
        rightShooter.setEnabled(true);
        pivot.setEnabled(true);

    }

    @Override
    public void initializeLogging() {
        // wow such empty...
    }

}
