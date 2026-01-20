package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.subsystems.template.TemplateSubsystem;

public class Indexer extends TemplateSubsystem {
    
    public Indexer() {
        super(
            "Indexer",
            0, //TODO
            SubsystemMode.VELOCITY,
            0
            );
        configureMotors(Constants.IndexerConstants.kSubsystemConfiguration);
    }
    @Override
    public void initializeLogging(){
        ShuffleboardTab indexerTab = Shuffleboard.getTab("Indexer");

        Reportable.addNumber(indexerTab, "Indexer RPM", () -> getCurrentValue(), Reportable.LOG_LEVEL.MINIMAL);
    }


}
