// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.RingDriveCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.subsystems.SuperSystem;
import frc.robot.util.Controller;

public class RobotContainer {
  public NerdDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  
  public SuperSystem superSystem;

  private final Controller driverController = new Controller(ControllerConstants.kDriverControllerPort, false);
  private final Controller operatorController = new Controller(ControllerConstants.kOperatorControllerPort,false);
  
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  
  private SwerveJoystickCommand swerveJoystickCommand;
  
  /**
   * The container for the robot. Contain
   * s subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try { swerveDrive = TunerConstants.createDrivetrain(); }
    catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    }

    if (Constants.USE_SUBSYSTEMS) {
       //add subsystems
      superSystem = new SuperSystem(swerveDrive);
      
    }

    initShuffleboard();
    initAutoChoosers();

    DriverStation.reportWarning("Initalization complete", false);
  }

  static boolean isRedSide = false;

  public static void refreshAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent())
      isRedSide = (alliance.get() == DriverStation.Alliance.Red);
  }

  public static boolean IsRedSide() {
    return isRedSide;
  }

  /**
   * Teleop commands configuration 
   * used in teleop mode.
   */
  public void initDefaultCommands_teleop() {
    swerveJoystickCommand = 
    new SwerveJoystickCommand(
      swerveDrive,
      () -> -driverController.getLeftY(), // Horizontal Translation
      () -> driverController.getLeftX(), // Vertical Translation
      () -> driverController.getRightX(), // Rotation
      () -> true, // robot oriented variable (true = field oriented)
      

      () -> false, // tow supplier
      () -> driverController.getTriggerRight(), // Precision/"Sniper Button"
      () -> false,
      () -> swerveDrive.getAbsoluteHeadingDegrees(), // TODO i have no clue if this is right // Turn to angle direction 
      () -> new Translation2d(  (driverController.getDpadUp()?1.0:0.0) - (driverController.getDpadDown()?1:0), 
                                (driverController.getDpadLeft()?1.0:0.0) - (driverController.getDpadRight()?1:0)) // DPad vector
    );
    swerveDrive.setDefaultCommand(swerveJoystickCommand);

    driverController.triggerLeft().whileTrue(new RingDriveCommand(
      swerveDrive,
      () -> -driverController.getLeftY(), // Horizontal Translation
      () -> driverController.getLeftX() // Vertical Translation)
      ));

    driverController.bumperRight().whileTrue(Commands.run(
      () -> swerveDrive.driveToTarget(new Pose2d())
    ));
  }

  public void initDefaultCommands_test() {
    initDefaultCommands_teleop();
  }

  public void configureBindings_teleop() {
    configureDriverBindings_teleop();
    configureOperatorBindings_teleop();
  }

  ///////////////////////
  // Driver bindings
  //////////////////////
  public void configureDriverBindings_teleop() {

    driverController.controllerLeft()
      .onTrue(Commands.runOnce(() -> swerveDrive.zeroFieldOrientation()));
    driverController.controllerRight()
      .onTrue(Commands.runOnce(() -> swerveDrive.resetRotation(Rotation2d.kZero)));
    // driverController.controllerRight()
    //   .onTrue(Commands.runOnce(() -> imu.zeroAbsoluteHeading()));

    if (Constants.USE_SUBSYSTEMS) {}
  }

  ///////////////////////
  // Operator bindings
  //////////////////////
  public void configureOperatorBindings_teleop() {

    if (Constants.USE_SUBSYSTEMS) {}
  }


  public void configureBindings_test() {}
  
  private void initAutoChoosers() {
    
    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");
    autosTab.add("Selected Auto", autoChooser);
  }
  
  public void initShuffleboard() {
    swerveDrive.initializeLogging();
  
    if (Constants.USE_SUBSYSTEMS) { 
      superSystem.initializeLogging();
    }
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command currentAuto = autoChooser.getSelected();
    return currentAuto;
  }

  public void disableAllMotors_Test()
  {
    swerveDrive.setBrake(true);
  }
  

  
}