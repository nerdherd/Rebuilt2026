// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.USE_VISION;

import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LoggingConstants;
import frc.robot.util.logging.NerdLog;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    DogLog.setOptions(LoggingConstants.LOGGING_CONFIGS);
    DogLog.setEnabled(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    // double startTime = MathSharedStore.getTimestamp();
    NerdLog.periodic();
    // DriverStation.reportWarning((MathSharedStore.getTimestamp() - startTime) + "s", false);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().getDefaultButtonLoop().clear();
    CommandScheduler.getInstance().cancelAll();
    
    if (Constants.USE_SUBSYSTEMS){
      m_robotContainer.superSystem.stop();
      m_robotContainer.superSystem.resetSubsystemValues();
    }

    m_robotContainer.swerveDrive.setVision(false);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.refreshAlliance();
    
    if (Constants.USE_SUBSYSTEMS) {
      m_robotContainer.superSystem.initialize();
      m_robotContainer.superSystem.reConfigureMotors();
    }
    
    m_robotContainer.swerveDrive.setVision(USE_VISION);
    if (USE_VISION) {
      CommandScheduler.getInstance().schedule(m_robotContainer.swerveDrive.resetPoseWithAprilTags(0.1));
      CommandScheduler.getInstance().schedule(Commands.runOnce(m_robotContainer.swerveDrive::setDriverHeadingForward));
    }

    // schedule the autonomous command (example)
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    // fix for elastic field pose not updating
    m_robotContainer.swerveDrive.setDefaultCommand(Commands.runOnce(Commands::none, m_robotContainer.swerveDrive));
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    RobotContainer.refreshAlliance();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    if (Constants.USE_SUBSYSTEMS) {
      m_robotContainer.superSystem.initialize();
    }
    
    m_robotContainer.swerveDrive.setVision(USE_VISION);
    if (USE_VISION) {
      CommandScheduler.getInstance().schedule(m_robotContainer.swerveDrive.resetPoseWithAprilTags(0.1));
      CommandScheduler.getInstance().schedule(Commands.runOnce(m_robotContainer.swerveDrive::setDriverHeadingForward));
    }

    m_robotContainer.initDefaultCommands_teleop();
    m_robotContainer.configureBindings_teleop();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.swerveDrive.setBrake(true);
    
    if (Constants.USE_SUBSYSTEMS) {
      m_robotContainer.superSystem.setNeutralMode(NeutralModeValue.Coast);
    }

    m_robotContainer.initDefaultCommands_test();
    m_robotContainer.configureBindings_test();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
