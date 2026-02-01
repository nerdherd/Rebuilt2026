package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.template.TemplateSubsystem;



public class Conveyor extends TemplateSubsystem {
     public Conveyor() {
        super(
            "Conveyor",
            ConveyorConstants.kMotor1ID, //TODO
            SubsystemMode.VOLTAGE,
            0, "CANivore"
            );
        configureMotors(ConveyorConstants.kSubsystemConfiguration);
    }

}
