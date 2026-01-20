package frc.robot.subsystems;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.subsystems.template.TemplateSubsystem;

public class Shooter extends TemplateSubsystem{
    
    public Shooter(String name, int motor1ID, int motor2ID){
        super(
            name,
            motor1ID, //TODO
            motor2ID,
            MotorAlignmentValue.Opposed,
            SubsystemMode.VELOCITY,
            0
        );
        configureMotors(Constants.ShooterConstants.kSubsystemConfiguration);
        
    }

    

    @Override
    public void initializeLogging(){
        ShuffleboardTab shooterTab = Shuffleboard.getTab("LeftShooter");

        Reportable.addNumber(shooterTab, "Shooter RPM", () -> getCurrentValue(), Reportable.LOG_LEVEL.MINIMAL);
    }
}
