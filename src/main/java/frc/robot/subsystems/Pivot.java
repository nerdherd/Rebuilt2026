package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.template.TemplateSubsystem;

public class Pivot extends TemplateSubsystem{
    public Pivot(){
        super("pivot",
            Constants.PivotConstants.kMotor1ID,
            Constants.PivotConstants.kMotor2ID,
            MotorAlignmentValue.Opposed,
            SubsystemMode.POSITION,
           0);
        
        configureMotors(Constants.PivotConstants.kSubsystemConfiguration);
    }

    public void initializeLogging(){
        
        Reportable.addNumber(shuffleboardTab, "Pivot Left", () -> getCurrentValue(), Reportable.LOG_LEVEL.MINIMAL);
        Reportable.addNumber(shuffleboardTab, "Pivot Right", () -> getCurrentValueMotor2(), Reportable.LOG_LEVEL.MINIMAL);
    
    }
}
