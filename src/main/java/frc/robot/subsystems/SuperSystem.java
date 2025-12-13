package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.template.TemplateSubsystem.LOG_LEVEL;

public class SuperSystem {
    public NerdDrivetrain swerveDrivetrain;
  
    //add new subsystems

    public SuperSystem(NerdDrivetrain swerveDrivetrain) {
        this.swerveDrivetrain = swerveDrivetrain;
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

    public void initShuffleboard(LOG_LEVEL priority){
    // redo the logging
    }

}
