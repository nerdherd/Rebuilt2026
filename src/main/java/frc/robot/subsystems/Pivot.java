package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.IntakeSlapdownConstants;
import frc.robot.subsystems.template.TemplateSubsystem;

public class Pivot extends TemplateSubsystem{
    public Pivot(){
        super("pivot",
            Constants.IntakeSlapdownConstants.kMotor1ID,
            SubsystemMode.POSITION,
           0, "rio");
        
        configureMotors(Constants.IntakeSlapdownConstants.kSubsystemConfiguration);
    }
}
