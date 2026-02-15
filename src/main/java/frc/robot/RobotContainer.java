// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ControllerConstants.kTurnToAngleFilter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Subsystems;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.autos.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.subsystems.SuperSystem;
import frc.robot.util.Controller;

public class RobotContainer {
  public NerdDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  
  public SuperSystem superSystem;

  private final Controller driverController = new Controller(ControllerConstants.kDriverControllerPort);
  private final Controller operatorController = new Controller(ControllerConstants.kOperatorControllerPort);
  
  private static boolean isRedSide = false;
  
  /**
   * The container for the robot. Contains
   * subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive = TunerConstants.createDrivetrain();

    if (Constants.USE_SUBSYSTEMS) { // add subsystems
      superSystem = new SuperSystem(swerveDrive);
    }
    
    Subsystems.init();
    initShuffleboard();
    Autos.initializeAutos();

    DriverStation.reportWarning("Initialization complete", false);
  }

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
    SwerveJoystickCommand swerveJoystickCommand =
    new SwerveJoystickCommand(
      swerveDrive,
      // Horizontal Translation
      () -> -driverController.getLeftY(), 
      // Vertical Translation
      () -> -driverController.getLeftX(), 
      // Turn
      () -> -driverController.getRightX(), 
      // use turn to angle
      () -> false,
      // turn to angle target direction, 0.0 to use manual
      () -> kTurnToAngleFilter.apply(driverController.getRightX(), driverController.getRightY()),
      // robot oriented adjustment (dpad)
      () -> new Translation2d(
        (driverController.getDpadUp() ? 1 : 0) - (driverController.getDpadDown() ? 1 : 0), 
        (driverController.getDpadLeft() ? 1 : 0) - (driverController.getDpadRight() ? 1 : 0)),
      // joystick drive field oriented
      () -> true, 
      // tow supplier
      () -> false, 
      // precision/programmer mode :)
      () -> driverController.getTriggerLeft()
    );
    
    swerveDrive.setDefaultCommand(swerveJoystickCommand);
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

    driverController.controllerLeft() // Set Drive Heading
      .onTrue(Commands.runOnce(() -> swerveDrive.setRobotHeadingForward()));

    driverController.controllerRight() // Set Pose Heading (pressed)
      .onTrue(Commands.runOnce(() -> swerveDrive.useMegaTag2 = false))
      .onFalse(Commands.runOnce(() -> swerveDrive.useMegaTag2 = true));

    // driverController.triggerLeft().whileTrue(new RingDriveCommand( // Ring Drive (held)
    //   swerveDrive,
    //   () -> -driverController.getRightY(), // Horizontal Translation
    //   () -> driverController.getLeftX() // Vertical Translation
    // ));

    if (Constants.USE_SUBSYSTEMS) {
      driverController.triggerRight()
        .onTrue(superSystem.intake())
        .onFalse(superSystem.stopIntaking());
    }
  }

  ///////////////////////
  // Operator bindings
  //////////////////////
  public void configureOperatorBindings_teleop() {
    if (Constants.USE_SUBSYSTEMS) {
      // operatorController.bumperRight()
      //   .onTrue(superSystem.intake())
      //   .onFalse(superSystem.stopIntaking());
      operatorController.bumperRight()
        .onTrue(superSystem.shoot())
        .onFalse(superSystem.stopShooting());
      operatorController.triggerRight()
        .onTrue(superSystem.spinUpFlywheel())
        .onFalse(superSystem.stopFlywheel());
      operatorController.triggerLeft()
        .onTrue(superSystem.intakeUp())
        .onFalse(superSystem.intakeDown());
    }
  }


  public void configureBindings_test() {}
  
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
    refreshAlliance();
    if (IsRedSide()) {
            return Autos.autonChooserRed.getSelected();
        } else {
            return Autos.autonChooserBlue.getSelected();
        }
  }
}
