package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.template.TemplateSubsystem;


public class Flywheel extends TemplateSubsystem{

    public Flywheel() {

        super("Top and Bottom Shooter", 53, 54, true, SubsystemMode.POSITION, 0);


        configureMotors(Constants.FlywheelConstants.kSubsystemConfiguration);

    }



    @Override
    public void initializeLogging() {

        ShuffleboardTab flywheeltab = Shuffleboard.getTab("flywheel");

        Reportable.addNumber(flywheeltab, "Top Shooter RPM", () -> getCurrentValue() , Reportable.LOG_LEVEL.MINIMAL);
        Reportable.addNumber(flywheeltab, "Bottom Shooter RPM", () -> getCurrentValueMotor2() , Reportable.LOG_LEVEL.MINIMAL);
    }
}