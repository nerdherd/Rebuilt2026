// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BiFunction;
import java.util.function.Function;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.subsystems.template.TemplateSubsystem.SubsystemMode;
import frc.robot.util.MultiProfiledPIDController;
import frc.robot.util.NerdyMath;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 // COMMENT ROBOT IDS INSTEAD OF DELETING

public final class Constants {

  /** current logging level of the robot's subsystems, @see Reportable.add... */
  public static final LOG_LEVEL ROBOT_LOG_LEVEL = LOG_LEVEL.MEDIUM;
  
  /** 
   * (hopefully) controls whether subsystem objects are used, swerve and others not counted
   * @see {@link frc.robot.subsystems.template.TemplateSubsystem TemplateSubsystem} 
   * @see {@link frc.robot.subsystems.SuperSystem SuperSystem}
   */
  public static final boolean USE_SUBSYSTEMS = true;
  /**
   * controls whether vision should be initialized
   */
  public static final boolean USE_VISION = true;

  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    // deadbands
    public static final double kTranslationDeadband = 0.1; // out of 1
    public static final double kRotationDeadband = 0.1; // out of 1
    public static final double kTurnToAngleDeadband = 0.5; // out of 1

    public static final double easePower = 3.0; // increase to further separate lower and higher values

    // returns a vector within the unit circle
    public static final BiFunction<Double, Double, Translation2d> kTranslationInputFilter = 
    (x, y) -> {
        x = NerdyMath.deadband(x, kTranslationDeadband);
        y = NerdyMath.deadband(y, kTranslationDeadband);
        if (x == 0.0 && y == 0.0) return new Translation2d();
        Translation2d dir = new Translation2d(x, y);
        double length = dir.getNorm();
        dir = dir.div(length);
        length = Math.min(1.0, length);
        length = Math.pow(length, easePower);
        return dir.times(length);
    };

    public static final Function<Double, Double> kRotationInputFilter = 
    (r) -> {
      return NerdyMath.deadband(r, kRotationDeadband);
    };

    public static final BiFunction<Double, Double, Double> kTurnToAngleFilter =
    (x, y) -> {
      if (NerdyMath.isPoseInsideCircleZone(0.0, 0.0, kTurnToAngleDeadband, x, y)) return Double.NaN;
      return Math.atan2(y, x);
    };
  }
  
  public static final class SwerveDriveConstants {
    //////////////////////////
    /// -- Drive Speeds -- ///
    //////////////////////////
    
    public static final double kDriveMaxVelocity = 5.0; // m/s
    public static final double kDrivePrecisionMultiplier = 0.25; // fractional
    
    public static final double kTurnMaxVelocity = 4.5; // rad/s
    public static final double kTurnPrecisionMultiplier = 0.5; // fractional
    
    public static final double kRobotOrientedVelocity = 1.0; // m/s
    
    ///////////////////////////
    /// -- Turn to Angle -- ///
    ///////////////////////////
    
    public static final double kTurnToAngleMaxVelocity = 4.5; // rad/s
    public static final PIDConstants kTurnToAnglePIDConstants = new PIDConstants(5.0, 0.0, 0.01);
    public static final Constraints kTurnToAngleTolerances = new Constraints(0.5, 1.0);

    ////////////////////////////////////////////
    /// -- NerdDrivetrain Swerve Requests -- ///
    ////////////////////////////////////////////

    /** Used for AutoBuilder configuration */
    public static final SwerveRequest.ApplyRobotSpeeds  kApplyRobotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
    /** Robot oriented controller */
    public static final SwerveRequest.RobotCentric      kRobotOrientedSwerveRequest = 
      new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.Position)
        ;
    /** Field oriented controller - use @see NerdDrivertrain#resetFieldOrientation() */
    public static final SwerveRequest.FieldCentric      kFieldOrientedSwerveRequest = 
      new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.Position)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        ;
    /** Field oriented controller - use @see NerdDrivertrain#resetFieldOrientation() */
    public static final SwerveRequest.SwerveDriveBrake  kTowSwerveRequest = 
      new SwerveRequest.SwerveDriveBrake()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.Position)
        ;

    ////////////////////////////////////////////
    /// -- Drive to Target Configurations -- ///
    ////////////////////////////////////////////

    /** @see NerdDrivetrain.driveToTarget() */
    public static final double kTargetDriveMaxLateralVelocity = 5.0;
    public static final PIDConstants kTargetDriveLateralPID = new PIDConstants(5.0, 0.0, 0.5);
    /** m/s and m/s/s @see NerdDrivetrain.driveToTarget() */
    public static final Constraints kTargetDriveLateralConstraints = new Constraints(kTargetDriveMaxLateralVelocity, kTargetDriveMaxLateralVelocity);
    public static final double kTargetDriveMaxRotationalVelocity = 9.4;
    public static final PIDConstants kTargetDriveRotationalPID = new PIDConstants(4.0, 0.0, 0.2);
    /** rad/s and rad/s/s @see NerdDrivetrain.driveToTarget() */
    public static final Constraints kTargetDriveRotationalConstraints = new Constraints(kTargetDriveMaxRotationalVelocity, kTargetDriveMaxRotationalVelocity);

    public static final MultiProfiledPIDController kTargetDriveController = new MultiProfiledPIDController()
      .add("x", kTargetDriveLateralPID, kTargetDriveLateralConstraints, 0.1, 0.1)
      .add("y", kTargetDriveLateralPID, kTargetDriveLateralConstraints, 0.1, 0.1)
      .add("r", kTargetDriveRotationalPID, kTargetDriveRotationalConstraints, 0.05, 0.2)
      .withContinuousInput("r", -Math.PI, Math.PI);

    public static enum FieldPositions {
      HUB_CENTER(4.626, 4.035, 0.0);
      //TODO Add field positions
      
      public Pose2d blue, red; // meters and degrees
      FieldPositions(double _blueX, double _blueY, double _blueHeadingDegrees) {
        blue = new Pose2d(new Translation2d(_blueX, _blueY), new Rotation2d(Units.degreesToRadians(_blueHeadingDegrees)));
        red = FlippingUtil.flipFieldPose(blue);
      }
    }
  }

  public static final class RingDriveConstants {
    public static final double kInitialDistance = 0.2; // m
    public static final double kDriveVelocity = 1.0; // m/s
    public static final double kMaximumDistance = 1.0; // m
    public static final double kMinimumDistance = 0.2; // m
    public static final double kRobotRotationOffset = Math.PI; // rad
  }

  public static final class PathPlannerConstants {
    public static final double kPP_P = 5.0; //6
    public static final double kPP_I = 0.0;
    public static final double kPP_D = 0.0;

    public static final PIDConstants kPPTranslationPIDConstants = new PIDConstants(kPP_P, kPP_I, kPP_D);

    public static final double kPP_ThetaP = 4.0; //3
    public static final double kPP_ThetaI = 0;
    public static final double kPP_ThetaD = 0.1;

    public static final PIDConstants kPPRotationPIDConstants = new PIDConstants(kPP_ThetaP, kPP_ThetaI, kPP_ThetaD);
  }

  public static final class VisionConstants {
    /** how many frames to skip in disabled, to prevent overheating */
    public static final int kDisabledThrottle = 100;

    public static enum Camera {
      // Example("limelight-ex", "10.6.87.XX:5802"),
      Charlie("limelight-charlie", "10.6.87.15:5802");

      public final String name, ip;
      Camera(String name, String ip) {
        this.name = name;
        this.ip = ip;
      }
    }
  }

  public static final class IntakeSlapdownConstants {
    public static final int kMotor1ID = 16;

    private static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
        .withKP(1.5)
        .withKI(0.0)
        .withKD(0.0)
      ;
    private static final MotorOutputConfigs kMotorOutputConfigs =
      new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);
      ;

    private static final MotionMagicConfigs kMotionMagicConfigs = 
      new MotionMagicConfigs()
        .withMotionMagicAcceleration(4)
        .withMotionMagicCruiseVelocity(2)
      ;
        
    public static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withMotorOutput(kMotorOutputConfigs)
        .withMotionMagic(kMotionMagicConfigs)
      ;

  }

  public static final class IntakeRollerConstants {
    public static final int kMotor1ID = 15;

    private static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.0)
      ;

    public static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
      ;
    
  }
  
  public static final class IndexerConstants {
    public static final int kMotor1ID = 25;

    public static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.0);

    public static final MotorOutputConfigs kMotorOutputConfigs =
      new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive);
        
    public static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withMotorOutput(kMotorOutputConfigs);
  }
   
  public static final class ConveyorConstants {
    public static final int kMotor1ID = 26;

    private static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.0)
      ;

    public static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
      .withSlot0(kSlot0Configs)
      ;
  }
  
  public static final class CounterRollerConstants {
    public static final int kMotor1ID = 37;

    private static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.0)
        .withKV(0.125)
      ;

    public static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
      .withSlot0(kSlot0Configs);
  }

  public static final class ShooterConstants {
    public static final int kMotor1ID = 35;
    public static final int kMotor2ID = 36;

    private static final Slot0Configs kSlot0ConfigsLeft = 
      new Slot0Configs()
        .withKP(0.05)
        .withKI(0.0)
        .withKD(0.0)
        .withKV(0.137035)
        ;
    
    private static final Slot0Configs kSlot0ConfigsRight = 
      new Slot0Configs()
        .withKP(0.05)
        .withKI(0.0)
        .withKD(0.0)
        .withKV(0.129204)
        ;
    
    private static final CurrentLimitsConfigs kCurrentLimitsConfigs = 
      new CurrentLimitsConfigs()
        .withStatorCurrentLimit(120)
        .withStatorCurrentLimitEnable(false)
      ;

    private static final MotionMagicConfigs kMotionMagicConfigs = 
      new MotionMagicConfigs()
        .withMotionMagicAcceleration(25)
      ;

    private static final MotorOutputConfigs kLeftMotorOutputConfigs =
      new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive);

    private static final MotorOutputConfigs kRightMotorOutputConfigs =
      new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive);

    private static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
        .withCurrentLimits(kCurrentLimitsConfigs)
        .withMotionMagic(kMotionMagicConfigs)
        ;

    public static final TalonFXConfiguration kLeftConfiguration = 
      kSubsystemConfiguration.clone()
        .withSlot0(kSlot0ConfigsLeft)
        .withMotorOutput(kLeftMotorOutputConfigs)
        ;

    public static final TalonFXConfiguration kRightConfiguration =
      kSubsystemConfiguration.clone()
        .withSlot0(kSlot0ConfigsRight)
        .withMotorOutput(kRightMotorOutputConfigs)
        ;
  }

  /** 
   * Container class to hold all subsystem objects.
   */
  public static final class Subsystems {
    public static final boolean useIntakeSlapdown = true;
    public static final TemplateSubsystem intakeSlapdown = (!USE_SUBSYSTEMS) ? null :
    new TemplateSubsystem(
        "Intake Slapdown", 
        IntakeSlapdownConstants.kMotor1ID, 
        SubsystemMode.POSITION, 
        0.0,
        useIntakeSlapdown)
      .configureMotors(IntakeSlapdownConstants.kSubsystemConfiguration);
    
    public static final boolean useIntakeRoller = true;
    public static final TemplateSubsystem intakeRoller = (!USE_SUBSYSTEMS) ? null :
    new TemplateSubsystem(
        "Intake Roller", 
        IntakeRollerConstants.kMotor1ID, 
        SubsystemMode.VELOCITY, 
        0.0,
        useIntakeRoller)
      .configureMotors(IntakeRollerConstants.kSubsystemConfiguration);
    
    public static final boolean useConveyor = true;
    public static final TemplateSubsystem conveyor = (!USE_SUBSYSTEMS) ? null :
    new TemplateSubsystem(
        "Conveyor", 
        ConveyorConstants.kMotor1ID, 
        SubsystemMode.VELOCITY, 
        0.0,
        useConveyor)
      .configureMotors(ConveyorConstants.kSubsystemConfiguration);    
    
    public static final boolean useIndexer = true;
    public static final TemplateSubsystem indexer = (!USE_SUBSYSTEMS) ? null :
    new TemplateSubsystem(
        "Indexer", 
        IndexerConstants.kMotor1ID, 
        SubsystemMode.VELOCITY, 
        0.0,
        useIndexer)
      .configureMotors(IndexerConstants.kSubsystemConfiguration);
    
    public static final boolean useCounterRoller = true;
    public static final TemplateSubsystem counterRoller = (!USE_SUBSYSTEMS) ? null :
    new TemplateSubsystem(
        "Counter Roller", 
        CounterRollerConstants.kMotor1ID, 
        SubsystemMode.VELOCITY, 
        0.0,
        useCounterRoller)
      .configureMotors(CounterRollerConstants.kSubsystemConfiguration);
    
    public static final boolean useShooter = true;
    public static final TemplateSubsystem shooterLeft = (!USE_SUBSYSTEMS) ? null :
    new TemplateSubsystem(
        "Shooter Left", 
        ShooterConstants.kMotor1ID,
        SubsystemMode.VELOCITY, 
        0.0,
        useShooter)
      .configureMotors(ShooterConstants.kLeftConfiguration);
    public static final TemplateSubsystem shooterRight = (!USE_SUBSYSTEMS) ? null :
    new TemplateSubsystem(
        "Shooter Right", 
        ShooterConstants.kMotor2ID, 
        SubsystemMode.VELOCITY, 
        0.0,
        useShooter)
      .configureMotors(ShooterConstants.kRightConfiguration);
    
    // literally just so the class actually loads
    // thanks java lazy loading
    public static void init() {} 
  }

  // public static class LEDConstants {
  //   public static final int CANdleID = 0;
  //   public static final int CANdleLength = 8;

  //   public static class Colors {
  //     public static final Color BLACK         = new Color(0.0, 0.0, 0.0); // shows up as nothing
  //     public static final Color WHITE         = new Color(1.0,1.0,1.0); 
  //     public static final Color RED           = new Color(1.0, 0.0, 0.0); 
  //     public static final Color GREEN         = new Color(0.0, 1.0, 0.0); 
  //     public static final Color BLUE          = new Color(0.0, 0.0, 1.0); 
  //     public static final Color NERDHERD_BLUE = new Color(0.132, 0.415, 1.0); // #071635 as base, brightened fully
  //   }
    
  //   public enum LEDStrips {
  //     ALL(0, CANdleLength),
  //     CANDLE(0,8),
  //     ;

  //     public int index, count;
  //     LEDStrips(int _index, int _count) {
  //       this.index = _index;
  //       this.count = _count;
  //     }
  //   }

  // }

  // ************************************** SUBSYSTEM CONSTANTS *************************************** //

  // Template Copy and Paste
  // public static final class SubsystemConstants {
  //   public static final int kMotor1ID = ;
  //   public static final int kMotor2ID = ;

  //   public static final Slot0Configs kSlot0Configs = 
  //     new Slot0Configs()
  //       .withKP(0.0)
  //     ;

  //   public static final TalonFXConfiguration kSubsystemConfiguration = 
  //     new TalonFXConfiguration()
  //     .withSlot0(kSlot0Configs)
  //     ;
  // }
  
  public static final class SuperSystemConstants {
    //TODO DO
  }
}
