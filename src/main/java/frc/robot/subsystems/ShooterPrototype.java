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

public class ShooterPrototype extends TemplateSubsystem {
    public ShooterPrototype(){
        super(
            "Shooter",
            53, 
            54, 
            MotorAlignmentValue.Opposed, 
            SubsystemMode.VELOCITY, 
            0
        );

        configureMotors(Constants.FlywheelConstants.ShooterPrototype);
    }

    @Override
    public void initializeLogging() {
        ShuffleboardTab shootertab = Shuffleboard.getTab("Shooter");

        Reportable.addNumber(shootertab, "Right Shooter RPM", () -> getCurrentValue() , Reportable.LOG_LEVEL.MINIMAL);
        Reportable.addNumber(shootertab, "Left Shooter RPM", () -> getCurrentValueMotor2() , Reportable.LOG_LEVEL.MINIMAL);
    }
}