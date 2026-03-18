// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Subsystems;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.autos.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.subsystems.SuperSystem;
import frc.robot.util.Controller;
import frc.robot.util.Controller.Type;
import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable.LOG_LEVEL;

public class RobotContainer {
  public NerdDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);
  
  public SuperSystem superSystem;

  private final Controller driverController = new Controller(ControllerConstants.kDriverControllerPort, Type.PS4);
  private final Controller operatorController = new Controller(ControllerConstants.kOperatorControllerPort, Type.PS4);
  private final Controller testController = new Controller(ControllerConstants.kTestControllerPort, Type.Xbox360);
  
  private static boolean isRedSide = false;
  
  /**
   * The container for the robot. Contains
   * subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive = TunerConstants.createDrivetrain();

    if (Constants.USE_SUBSYSTEMS) { // add subsystems
      superSystem = new SuperSystem(swerveDrive);
      Autos.initNamedCommands(superSystem, swerveDrive);
      Autos.initEventMarkers(superSystem, swerveDrive);
    }
    
    Subsystems.init();
    Autos.initAutoChooser();
    initializeLogging();

    NerdLog.reportInfo("Initialization Complete");
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
      () -> driverController.getBumperRight() || driverController.getBumperLeft(),
      // turn to angle target direction, 0.0 to use manual
      () -> (driverController.getBumperRight()) ? swerveDrive.angleToPose(FieldPositions.HUB_CENTER) : 0.0,
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
    swerveDrive.removeDefaultCommand();
    // initDefaultCommands_teleop();
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
      .onTrue(Commands.runOnce(() -> swerveDrive.recalibrateGyroMT1()));

    // driverController.triggerLeft().whileTrue(new RingDriveCommand( // Ring Drive (held)
    //   swerveDrive,
    //   () -> -driverController.getRightY(), // Horizontal Translation
    //   () -> driverController.getLeftX() // Vertical Translation
    // ));

    if (Constants.USE_SUBSYSTEMS) {
      driverController.triggerRight()
        .onTrue(superSystem.intake())
        .onFalse(superSystem.stopIntaking());

      // driverController.buttonDown()
      //   .whileTrue(superSystem.shootWithTuning())
      //   .onFalse(superSystem.stopFlywheel());
      // driverController.buttonLeft()
      //   .whileTrue(superSystem.shootWithCondition())
      //   .onFalse(superSystem.stopShooting());

      // driverController.bumperLeft()
      //   .whileTrue(superSystem.climbUp())
      //   .onFalse(superSystem.stopClimb());
      // driverController.buttonRight()
      //   .whileTrue(superSystem.climbDown())
      //   .onFalse(superSystem.stopClimb());
    }
  }

  ///////////////////////
  // Operator bindings
  //////////////////////
  public void configureOperatorBindings_teleop() {

    if (Constants.USE_SUBSYSTEMS) {
      operatorController.bumperLeft()
        .onTrue(superSystem.intake())
        .onFalse(superSystem.stopIntaking());
      operatorController.controllerLeft()
        .onTrue(superSystem.intakeHold());

      operatorController.triggerRight()
        .whileTrue(superSystem.shootWithDistanceAndFeed());
        // .whileTrue(superSystem.shootWithTuning()) // USE ELASTIC
        // .onTrue(superSystem.spinUpFlywheel())
        // .onFalse(superSystem.stopFlywheel());
      operatorController.triggerLeft()
        .whileTrue(superSystem.spinUpFlywheel())
        .onFalse(superSystem.stopFlywheel());
      operatorController.bumperRight()
        .whileTrue(superSystem.shootWithCondition())
        .onFalse(superSystem.stopShooting());
        
      operatorController.buttonRight()
        .onTrue(superSystem.outtake())
        .onFalse(superSystem.stopIntaking());
      operatorController.buttonDown()
        .onTrue(superSystem.reverseConveyor())
        .onFalse(superSystem.stopConveyor());
      
      operatorController.dpadUp()
        .whileTrue(superSystem.climbUp())
        .onFalse(superSystem.stopClimb());
        
      operatorController.dpadDown()
        .whileTrue(superSystem.climbDown())
        .onFalse(superSystem.stopClimb());
    }
  }

  public void configureBindings_test() {

    testController.buttonRight()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Right Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Right Test", "bye")));
    testController.buttonDown()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Down Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Down Test", "bye")));
    testController.buttonUp()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Up Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Up Test", "bye")));
    testController.buttonLeft()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Left Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Left Test", "bye")));

    testController.bumperLeft()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Bumper L Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Bumper L Test", "bye")));
    testController.bumperRight()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Bumper R Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Bumper R Test", "bye")));
    
    testController.triggerLeft()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Trigger L Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Trigger L Test", "bye")));
    testController.triggerRight()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Trigger R Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Trigger R Test", "bye")));

   testController.dpadUp()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Up Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Up Test", "bye")));
    testController.dpadRight()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Right Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Right Test", "bye")));
    testController.dpadDown()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Down Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Down Test", "bye")));
    testController.dpadLeft()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Left Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Left Test", "bye")));

    testController.controllerLeft()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Controller Left Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Controller Left Test", "bye")));
    testController.controllerRight()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Controller Right Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Controller Right Test", "bye")));
    
    testController.joystickLeft()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Left Joy Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Left Joy Test", "bye")));
    testController.joystickRight()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Right Joy Test", "hi")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Right Joy Test", "bye")));
      
  }
  
  private StringSubscriber printLog = null;
  public void initializeLogging() {
    if (printLog == null) printLog = DogLog.tunable("Robot/Print", "", (val) -> NerdLog.reportInfo(val));
    NerdLog.logData("Robot/PDP", pdp, LOG_LEVEL.ALL);
    
    swerveDrive.initializeLogging();
    if (Constants.USE_SUBSYSTEMS) { 
      superSystem.initializeLogging();
    }
    
    NerdLog.logData("Robot/Command Scheduler", CommandScheduler.getInstance(), LOG_LEVEL.ALL);
    NerdLog.logNumber("Robot/RAM Usage", () -> (double)Runtime.getRuntime().freeMemory(), LOG_LEVEL.MEDIUM);
    NerdLog.reportLogCount();
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.autoChooser.getSelected();
  }

  public void disableAllMotors_Test() {
    swerveDrive.setBrake(true);
  }
}
