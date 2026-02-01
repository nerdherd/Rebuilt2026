package frc.robot.subsystems;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.subsystems.template.TemplateSubsystem;

public class Shooter extends TemplateSubsystem{
    
    public Shooter(){
        super(
            "Shooter",
            Constants.ShooterConstants.kMotor1ID, //TODO
            Constants.ShooterConstants.kMotor2ID,
            MotorAlignmentValue.Opposed,
            SubsystemMode.VELOCITY,
            0, "rio"
        );
        configureMotors(Constants.ShooterConstants.kSubsystemConfiguration);
        
    }

    

    @Override
    public void initializeLogging(){

        Reportable.addNumber(shuffleboardTab, "Shooter RPM", () -> getCurrentValue(), Reportable.LOG_LEVEL.MINIMAL);
    }
}
