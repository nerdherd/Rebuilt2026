package frc.robot.subsystems;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.subsystems.template.TemplateSubsystem;

public class Intake extends TemplateSubsystem{

    public Intake() {
        super(
            "Intake",
            Constants.IntakeConstants.kMotor1ID,
            SubsystemMode.VELOCITY,
            0
            );
        configureMotors(Constants.IntakeConstants.kSubsystemConfiguration);
    }
    @Override
    public void initializeLogging(){
        Reportable.addNumber(shuffleboardTab, "Intake Left", () -> getCurrentValue(), Reportable.LOG_LEVEL.MINIMAL);
        Reportable.addNumber(shuffleboardTab, "Intake Right", () -> getCurrentValueMotor2(), Reportable.LOG_LEVEL.MINIMAL);
    }
    
}
