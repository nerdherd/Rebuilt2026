package frc.robot.subsystems;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.subsystems.template.TemplateSubsystem;

public class Intake extends TemplateSubsystem{

    public Intake() {
        super(
            "Intake",
            0,
            SubsystemMode.VELOCITY,
            0
            );
        configureMotors(Constants.IntakeConstants.kSubsystemConfiguration);
    }
    @Override
    public void initializeLogging(){
        ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

        Reportable.addNumber(intakeTab, "Intake RPM", () -> getCurrentValue(), Reportable.LOG_LEVEL.MINIMAL);
    }
    
}
