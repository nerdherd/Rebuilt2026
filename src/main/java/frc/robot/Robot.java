// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.USE_VISION;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private static DriverStation.Alliance alliance;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    DataLogManager.start("/media/sda1/logs");
    DataLogManager.logNetworkTables(true);
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().getDefaultButtonLoop().clear();
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.swerveDrive.setVision(false);
    
    if (Constants.USE_SUBSYSTEMS){
      m_robotContainer.superSystem.stop();
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    if (DriverStation.getAlliance().isPresent()) alliance = DriverStation.getAlliance().get();
    m_robotContainer.swerveDrive.setVision(USE_VISION); 
    m_robotContainer.swerveDrive.resetRotation(new Rotation2d(0.0 + 180.0)); // TODO consider auto field orientation
    // m_robotContainer.swerveDrive.enableLimeLight(); TODO

    if (Constants.USE_SUBSYSTEMS) {
      m_robotContainer.superSystem.setNeutralMode(NeutralModeValue.Brake);
      m_robotContainer.superSystem.initialize();
    }
  // schedule the autonomous command (example) 
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (DriverStation.getAlliance().isPresent()) alliance = DriverStation.getAlliance().get();
    m_robotContainer.swerveDrive.setVision(USE_VISION);
    // TODO decide whether to keep this
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (Constants.USE_SUBSYSTEMS) {
      m_robotContainer.superSystem.setNeutralMode(NeutralModeValue.Brake);
      m_robotContainer.superSystem.initialize();
    }

    m_robotContainer.configureBindings_teleop();
    m_robotContainer.initDefaultCommands_teleop();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    if (Constants.USE_SUBSYSTEMS) {
      m_robotContainer.superSystem.setNeutralMode(NeutralModeValue.Coast);
      m_robotContainer.superSystem.initialize();
    }
    // m_robotContainer.superSystem.initialize();
    m_robotContainer.initDefaultCommands_test();
    m_robotContainer.configureBindings_test();

    m_robotContainer.disableAllMotors_Test();

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

  public static DriverStation.Alliance getAlliance() {
        return alliance;
    }
}
