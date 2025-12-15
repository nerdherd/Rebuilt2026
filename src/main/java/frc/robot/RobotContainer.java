// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import com.ctre.phoenix6.controls.VoltageOut;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.autos.PreloadTaxi;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.subsystems.SuperSystem;
import frc.robot.util.BannerSensor;
import frc.robot.util.Controller;

public class RobotContainer {
  public NerdDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  
  public BannerSensor floorSensor;
  public SuperSystem superSystem;

  private final Controller driverController = new Controller(ControllerConstants.kDriverControllerPort, false);
  private final Controller operatorController = new Controller(ControllerConstants.kOperatorControllerPort,false);
  
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  static boolean isRedSide = false;
  
  private SwerveJoystickCommand swerveJoystickCommand;
  
  // For logging wrist
  public final VoltageOut voltageRequest = new VoltageOut(0);
  public double voltage = 0;
  public double desiredAngle = 0.0; //164, 99.8

  public double desiredRotation = 0.0;//ElevatorConstants.kElevatorPivotStowPosition; -1.6

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
      () -> false, // robot oriented variable (false = field oriented)
      () -> false, // tow supplier
      () -> driverController.getTriggerRight(), // Precision/"Sniper Button"
      () -> false,
      () -> swerveDrive.getAbsoluteHeadingDegrees(), // TODO i have no clue if this is right // Turn to angle direction 
      () -> new Translation2d(   (driverController.getDpadUp()?1.0:0.0) - (driverController.getDpadDown()?1:0), 
                                        (driverController.getDpadRight()?1.0:0.0) - (driverController.getDpadLeft()?1:0)) // DPad vector
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

    driverController.controllerLeft()
      .onTrue(Commands.runOnce(() -> swerveDrive.zeroFieldOrientation()));
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

    // try { // ide displayed error fix
      // bottom2Piece = new Bottom2Piece(swerveDrive, intakeRoller, elevator, "Bottom2Piece");
      // pathOnlyBottom2Piece = new PathOnlyBottom2Piece(swerveDrive, "PathOnlyBottom2Piece");
    // } catch (Exception e) {
    //   DriverStation.reportError("IOException for Bottom2Piece", e.getStackTrace());
    //   return;
    // } 

    try { // fix for vendordeps not importing
    // PathPlannerPath S4R3 = PathPlannerPath.fromPathFile("S4R3");

  	// List<String> paths = AutoBuilder.getAllAutoNames();
    
    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");
    autosTab.add("Selected Auto", autoChooser);
    // autoChooser.setDefaultOption("Do Nothing", Commands.none());
    
    autoChooser.setDefaultOption("PreloadTaxi", new PreloadTaxi(swerveDrive, "TaxiPreload", superSystem));
    autoChooser.addOption("PreloadTaxi", new PreloadTaxi(swerveDrive, "TaxiPreload", superSystem));
    autoChooser.addOption("Taxi", AutoBuilder.buildAuto("Taxi"));
    // autoChooser.addOption("TaxiLeft", AutoBuilder.buildAuto("S1Taxi"));
    // autoChooser.addOption("TaxiRight", AutoBuilder.buildAuto("S7Taxi"));
    
    // autoChooser.addOption("2PieceLeftOffset", new TwoPieceOffset(swerveDrive, "TopTwoPieceOffset", superSystem));
    // autoChooser.addOption("2PieceLeft", new TwoPiece(swerveDrive, "TopTwoPiece", superSystem));
    // autoChooser.addOption("2PieceRightOffset", new TwoPieceOffset(swerveDrive, "BottomTwoPieceOffset", superSystem));
    // autoChooser.addOption("2PieceGround", new TwoPieceGround(swerveDrive, "BottomTwoPieceGround", superSystem));
    // autoChooser.addOption("2PieceRight", new TwoPiece(swerveDrive, "BottomTwoPiece", superSystem));
    
    // autoChooser.addOption("2PiecePathOnly", new TwoPiecePath(swerveDrive, "TopTwoPiece", superSystem));
    
    // autoChooser.addOption("Bottom 3 Piece", bottom3Piece);
    // autoChooser.addOption("Bottom 4 Piece", bottom4Piece);
    

    } catch (Exception e) { SmartDashboard.putBoolean("Auto Error", true); }
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

  public void DisableAllMotors_Test()
  {
    swerveDrive.setBrake(true);
  }
  

  
}