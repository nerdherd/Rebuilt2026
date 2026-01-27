package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SuperSystem implements Reportable {
    public NerdDrivetrain swerveDrivetrain;
    
    public CounterRoller counterRoller;
    public ShooterPrototype shooterPrototype;
    public IndexerPrototype indexerPrototype;

  
    //add new subsystems

    public SuperSystem(NerdDrivetrain swerveDrivetrain, CounterRoller counterRoller, ShooterPrototype shooterPrototype, IndexerPrototype indexerPrototype) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.counterRoller = counterRoller;
        this.shooterPrototype = shooterPrototype;
        this.indexerPrototype = indexerPrototype;
    }

    // ------------------------------------ subsystems ------------------------------------ //
    public void reConfigureMotors() {
        // redo
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

    }
    public Command shoot(){
        return Commands.parallel(
            counterRoller.setDesiredValueCommand(60),
            counterRoller.setEnabledCommand(true)
        );
    }

    public Command spinUp(){
        return Commands.parallel(
            shooterPrototype.setDesiredValueCommand(60),
            shooterPrototype.setEnabledCommand(true)
        );
    }
    public Command stopShooting(){
        return Commands.parallel(
             shooterPrototype.setDesiredValueCommand(0),
            shooterPrototype.setEnabledCommand(false),
            counterRoller.setDesiredValueCommand(0),
            counterRoller.setEnabledCommand(false)
        );
    }
    public Command index(){
        return Commands.parallel(
            indexerPrototype.setDesiredValueCommand(20),
            indexerPrototype.setEnabledCommand(true)
        );
    }

    public Command stopIndexing(){
        return Commands.parallel(
            indexerPrototype.setDesiredValueCommand(0),
            indexerPrototype.setEnabledCommand(false)
        );
    }
    @Override
    public void initializeLogging() {
        // wow such empty...
    }


}
