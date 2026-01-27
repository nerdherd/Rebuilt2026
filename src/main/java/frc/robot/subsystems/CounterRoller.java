package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.template.TemplateSubsystem;

public class CounterRoller extends TemplateSubsystem {

    public CounterRoller(){
        super(
            "CounterRoller", 
            11, 
            12,
            MotorAlignmentValue.Opposed,
            SubsystemMode.VELOCITY, 
            0);
        
        configureMotors(Constants.CounterRoller.kSubsystemConfiguration);
    }

    @Override 
    public void initializeLogging() { 
        ShuffleboardTab counterRollertab = Shuffleboard.getTab("CounterRoller");

        Reportable.addNumber(counterRollertab, "Indexer RPM", () -> getCurrentValue() , Reportable.LOG_LEVEL.MINIMAL);
    }
}
