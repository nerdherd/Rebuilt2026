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

public class IndexerPrototype extends TemplateSubsystem {

    public IndexerPrototype(){
        super(
            "Indexer", 
            Constants.IndexerConstants.kMotor1, 
            SubsystemMode.VELOCITY, 
            0);
        
        configureMotors(Constants.IndexerConstants.kSubsystemConfiguration);
    }

    @Override
    public void initializeLogging() {
        ShuffleboardTab indexertab = Shuffleboard.getTab("Indexer");

        Reportable.addNumber(indexertab, "Indexer RPM", () -> getCurrentValue() , Reportable.LOG_LEVEL.MINIMAL);
    }
}

