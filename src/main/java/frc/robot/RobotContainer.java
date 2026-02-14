// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.autos.Autos;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.subsystems.SuperSystem;
import frc.robot.util.Controller;
import frc.robot.util.Controller.Type;

public class RobotContainer {
  public NerdDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  
  public SuperSystem superSystem;

  private final Controller driverController = new Controller(ControllerConstants.kDriverControllerPort, Type.GuliKit, true, true);
  private final Controller operatorController = new Controller(ControllerConstants.kOperatorControllerPort, Type.GuliKit, true, true); 
  private SwerveJoystickCommand swerveJoystickCommand;
  
  /**
   * The container for the robot. Contains
   * subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive = TunerConstants.createDrivetrain();

    //Named Command Initialization

    NamedCommands.registerCommand("Wait", Commands.waitSeconds(1));

    if (Constants.USE_SUBSYSTEMS) {
      superSystem = new SuperSystem(swerveDrive);
    }
    
    initShuffleboard();
    Autos.initializeAutos();

    DriverStation.reportWarning("Initalization complete", false);
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
      () -> true, // field oriented variable (true = field oriented)
      () -> false, // tow supplier
      () -> driverController.getTriggerLeft(), // Precision/"Sniper Button"
      () -> false,
      () -> swerveDrive.getAbsoluteHeadingDegrees(), // TODO i have no clue if this is right // Turn to angle direction 
      () -> new Translation2d(  (driverController.getDpadUp()?1.0:0.0) - (driverController.getDpadDown()?1:0), 
                                (driverController.getDpadLeft()?1.0:0.0) - (driverController.getDpadRight()?1:0)) // DPad vector
    );
    swerveDrive.setDefaultCommand(swerveJoystickCommand);

    // driverController.triggerLeft().whileTrue(new RingDriveCommand(
    //   swerveDrive,
    //   () -> -driverController.getRightY(), // Horizontal Translation
    //   () -> driverController.getLeftX() // Vertical Translation
    //   ));

    // driverController.bumperRight().whileTrue(Commands.run( // DriveToTarget test
    //   () -> swerveDrive.driveToTarget(new Pose2d())
    // ));
  }

  public void initDefaultCommands_test() {
    
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
      .onTrue(Commands.runOnce(() -> swerveDrive.zeroOperatorHeading()));
    driverController.controllerRight()
      .onTrue(swerveDrive.resetPoseWithAprilTags(0.1));
      // .onTrue(Commands.runOnce(() -> swerveDrive.useMegaTag2 = false));
    // driverController.controllerRight()

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
      operatorController.bumperRight()
        .onTrue(superSystem.intake())
        .onFalse(superSystem.stopIntaking());
      operatorController.bumperLeft()
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


  public void configureBindings_test() {

    driverController.buttonRight()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button A Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button A Test", "bye")));
    driverController.buttonDown()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button B Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button B Test", "bye")));
    driverController.buttonUp()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button X Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button X Test", "bye")));
    driverController.buttonLeft()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Y Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Y Test", "bye")));

    driverController.bumperLeft()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Bumper L Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Bumper L Test", "bye")));
    driverController.bumperRight()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Bumper R Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Bumper R Test", "bye")));
    
      driverController.triggerLeft()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Trigger ZL Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Trigger ZL Test", "bye")));
    driverController.triggerRight()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Trigger ZR Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Trigger ZR Test", "bye")));

    driverController.dpadUp()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Up Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Up Test", "bye")));
    driverController.dpadRight()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Right Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Right Test", "bye")));
    driverController.dpadDown()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Down Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Down Test", "bye")));
    driverController.dpadLeft()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Left Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Left Test", "bye")));

    driverController.controllerLeft()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Minus Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Minus Test", "bye")));
    driverController.controllerRight()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Plus Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Plus Test", "bye")));
    
      driverController.joystickLeft()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Left Joy Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Left Joy Test", "bye")));
    driverController.joystickRight()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Right Joy Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Right Joy Test", "bye")));
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
    if (Robot.getAlliance().equals(DriverStation.Alliance.Red))
      return Autos.autonChooserRed.getSelected();
    else return Autos.autonChooserBlue.getSelected();
  }

  public void disableAllMotors_Test() {
    swerveDrive.setBrake(true);
  }

}