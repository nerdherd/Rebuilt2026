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

public class IntakePrototype extends TemplateSubsystem {

    public IntakePrototype(){


        super("Intake", 53, 54, MotorAlignmentValue.Aligned, SubsystemMode.VELOCITY, 0);


        configureMotors(Constants.FlywheelConstants.kSubsystemConfiguration);
    }

      @Override
    public void initializeLogging() {

        ShuffleboardTab intaketab = Shuffleboard.getTab("intake");

        Reportable.addNumber(intaketab, "Right Shooter RPM", () -> getCurrentValue() , Reportable.LOG_LEVEL.MINIMAL);
        Reportable.addNumber(intaketab, "Left Shooter RPM", () -> getCurrentValueMotor2() , Reportable.LOG_LEVEL.MINIMAL);
    }
}