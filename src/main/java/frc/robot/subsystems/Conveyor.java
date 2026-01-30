package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.subsystems.template.TemplateSubsystem;

public class Conveyor extends TemplateSubsystem{

    public Conveyor(){
        super("Conveyor",
         Constants.CounterRollerConstants.kMotor1ID, //TODO
         SubsystemMode.VELOCITY,
         0
           );
        configureMotors(Constants.ConveyorConstants.kSubsystemConfiguration);
    }

    @Override
    public void initializeLogging(){
       Reportable.addNumber(shuffleboardTab, "CounterRoller RPM", () -> getCurrentValue(), Reportable.LOG_LEVEL.MINIMAL);
    }
    
}
