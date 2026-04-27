// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.SwerveDriveConstants.kRobotOrientedVelocity;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Subsystems;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.autos.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.subsystems.SuperSystem;
import frc.robot.util.controller.Controller;
import frc.robot.util.controller.Controller.Type;
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
      superSystem.initializeLEDs();
      Autos.initNamedCommands(superSystem, swerveDrive);
    }
    
    Subsystems.init();
    Autos.initAutoChooser();
    initializeLogging();

    NerdLog.get().reportInfo("Initialization Complete");
  }

  public static void refreshAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent())
      isRedSide = (alliance.get() == DriverStation.Alliance.Red);
  }

  public static boolean IsRedSide() {
    return isRedSide;
  }

  public static double kOffset = 0.05;
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
      () -> driverController.getBumperRight(),
      // turn to angle target direction, 0.0 to use manual
      () -> swerveDrive.angleToLookAheadPose(FieldPositions.HUB_CENTER, ShooterConstants.kLookAheadFactor) + kOffset,
      // robot oriented adjustment (dpad)
      () -> new Translation2d(
        (((driverController.getDpadUp() && !driverController.getBumperRight()) ? 1 : 0) - (driverController.getDpadDown() ? 1 : 0)) * kRobotOrientedVelocity, 
        ((driverController.getDpadLeft() ? 1 : 0) - (driverController.getDpadRight() ? 1 : 0)) * 1.5)
        .rotateBy((!driverController.getBumperRight()) ? Rotation2d.kZero : 
            Rotation2d.fromRadians(swerveDrive.angleToLookAheadPose(FieldPositions.HUB_CENTER, ShooterConstants.kLookAheadRingDriveFactor) - swerveDrive.angleToLookAheadPose(FieldPositions.HUB_CENTER, ShooterConstants.kLookAheadFactor) - kOffset)),
      // joystick drive field oriented
      () -> true, 
      // tow supplier
      () -> driverController.getBumperLeft(), 
      // precision/programmer mode :)
      () -> driverController.getTriggerLeftAxis()
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
      // driverController.buttonUp()
      //   .whileTrue(superSystem.shootWithDistance())
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
      operatorController.controllerLeft()
        .onTrue(superSystem.intakeHoldTeleop());
        // .onFalse(superSystem.stopIntakeHold());
      operatorController.controllerRight()
        .onTrue(superSystem.intakeUp())
        .onFalse(superSystem.stopIntakeHold());
      operatorController.bumperLeft()
        .onTrue(superSystem.intake())
        .onFalse(superSystem.stopIntaking());

      operatorController.triggerRight()
        .whileTrue(superSystem.shootWithDistance())
        // .whileTrue(superSystem.shootWithTuning()) // USE ELASTIC
        // .onTrue(superSystem.spinUpFlywheel())
        .onFalse(superSystem.stopFlywheel());
      operatorController.triggerLeft()
        .whileTrue(superSystem.spinUpFlywheel())
        .onFalse(superSystem.stopFlywheel());
      operatorController.bumperRight()
        .whileTrue(superSystem.shootWithCondition())
        .onFalse(superSystem.stopShooting());
        
      operatorController.buttonUp()
        .onTrue(superSystem.spinUpFlywheelFeeding())
        .onFalse(superSystem.stopFlywheel());
      operatorController.buttonRight()
        .onTrue(superSystem.outtake())
        .onFalse(superSystem.stopIntaking());
      operatorController.buttonDown()
        .onTrue(superSystem.reverseConveyor())
        .onFalse(superSystem.stopConveyor());
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
  
  public StringSubscriber printLog = null;
  public void initializeLogging() {
    if (printLog == null) printLog = DogLog.tunable("Print", "", (value) -> NerdLog.get().reportInfo("" + value));
    NerdLog.get().logData("Robot/PDP", pdp, LOG_LEVEL.ALL);
    
    swerveDrive.initializeLogging();
    if (Constants.USE_SUBSYSTEMS) { 
      superSystem.initializeLogging();
    }

    NerdLog.get().logData("Robot/Command Scheduler", CommandScheduler.getInstance(), LOG_LEVEL.MEDIUM);
    NerdLog.get().logNumber("Robot/RAM Usage", () -> (double)Runtime.getRuntime().freeMemory(), LOG_LEVEL.MEDIUM);
    NerdLog.getNT().logNumber("Match Info/Shift Time", () -> {shiftTime = allianceShiftTime(); return shiftTime;}, LOG_LEVEL.MINIMAL);
    NerdLog.get().reportLogCount();
    NerdLog.getNT().reportLogCount();
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

  private static boolean gameEnded = false;
  public static double shiftTime = 0.0;
  /**
   * Displays a countdown for alliance shifts. NOT 100% ACCURATE
   * @return the number of seconds in the current phase, and the phase name
   */
  public static double allianceShiftTime() {
    // if (!DriverStation.isFMSAttached()) { DogLog.forceNT.log("Match Info/Shift Name", "DriverStation not attached"); return 0.0; };
    boolean wonAuto = true;
    if (Constants.ROBOT_LOG_LEVEL == LOG_LEVEL.MEDIUM) {
      String data = DriverStation.getGameSpecificMessage();
      if (!data.isEmpty()) switch (data.charAt(0)) {
        case 'B': wonAuto = !isRedSide; break;
        case 'R': wonAuto = isRedSide; break;
        default: break;
      } 
      DogLog.forceNt.log("Match Info/Won Auto?", wonAuto);
    }

    double time = DriverStation.getMatchTime();
    DogLog.forceNt.log("Match Info/time", time);

    if (DriverStation.isAutonomous()) {
      if (time < 0.0) { DogLog.forceNt.log("Match Info/Shift Name", (gameEnded) ? "Good Job Team!" : "Get Ready..."); return 0.0; }
      DogLog.forceNt.log("Match Info/Shift Name", "Auto");
      gameEnded = false;
      return time;
    } else if (DriverStation.isTeleop()) {
      if (time < 0.0) { DogLog.forceNt.log("Match Info/Shift Name", (gameEnded) ? "Good Job Team!" : "Good Luck! -nerdherd"); return 0.0; }
      else if (time >= 130.0) { DogLog.forceNt.log("Match Info/Shift Name", "Transition"); return time - 130; } // transition
      else if (time >= 30.0) { 
        int shift = (int)((130 - time) / 25) + 1; 
        DogLog.forceNt.log("Match Info/Shift Name", "Shift " + shift + " " + (((shift % 2 == 1) == wonAuto) ? "Feeding" : "Scoring")); return (time - 30) % 25; 
      } // shifts 1-4
      else { DogLog.forceNt.log("Match Info/Shift Name", "Endgame"); if (time <= 1.0) gameEnded = true; return time; } // endgame
    } else { DogLog.forceNt.log("Match Info/Shift Name", "Inactive"); return 0.0; }
  }
}
