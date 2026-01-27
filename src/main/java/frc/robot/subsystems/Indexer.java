package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.subsystems.template.TemplateSubsystem;

public class Indexer extends TemplateSubsystem {
    
    public Indexer() {
        super(
            "Indexer",
            Constants.IndexerConstants.kMotor1ID, //TODO
            SubsystemMode.VELOCITY,
            0
            );
        configureMotors(Constants.IndexerConstants.kSubsystemConfiguration);
    }
    @Override
    public void initializeLogging(){
        Reportable.addNumber(shuffleboardTab, "Indexer RPM", () -> getCurrentValue(), Reportable.LOG_LEVEL.MINIMAL);
    }


}
