// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BiFunction;
import java.util.function.Function;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.subsystems.template.TemplateSubsystem.SubsystemMode;
import frc.robot.util.MultiProfiledPIDController;
import frc.robot.util.NerdyMath;
import frc.robot.util.Translation2dSlewRateLimiter;
import frc.robot.util.logging.Reportable.LOG_LEVEL;

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
    public static final int kTestControllerPort = 2;

    // deadbands
    public static final double kTranslationDeadband = 0.1; // out of 1
    public static final double kRotationDeadband = 0.1; // out of 1
    public static final double kTurnToAngleDeadband = 0.5; // out of 1

    public static final double kInputAcceleration = 0.75; // unit/s, on the scale of a unit circle/fractions
    public static final double kEasePower = 3.0; // increase to further separate lower and higher values

    public static final Translation2dSlewRateLimiter kTranslationInputRateLimiter = new Translation2dSlewRateLimiter(kInputAcceleration);
    
    // returns a vector within the unit circle
    public static final BiFunction<Double, Double, Translation2d> kTranslationInputFilter = 
    (x, y) -> {
        x = NerdyMath.deadband(x, kTranslationDeadband);
        y = NerdyMath.deadband(y, kTranslationDeadband);
        if (x == 0.0 && y == 0.0) {
          kTranslationInputRateLimiter.reset();
          return Translation2d.kZero;
        }
        Translation2d dir = new Translation2d(x, y);
        double length = dir.getNorm();
        dir = dir.div(length);
        length = Math.min(1.0, length);
        length = Math.pow(length, kEasePower);
        return kTranslationInputRateLimiter.calculate(dir.times(length));
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
    public static final double kDrivePrecisionMultiplier = 0.5; // fractional
    
    public static final double kTurnMaxVelocity = 6.0; // rad/s
    public static final double kTurnPrecisionMultiplier = 0.5; // fractional
    
    public static final double kRobotOrientedVelocity = 1.5; // m/s
    
    ///////////////////////////
    /// -- Turn to Angle -- ///
    ///////////////////////////
    
    public static final double kTurnToAngleMaxVelocity = 5.5; // rad/s
    public static final PIDConstants kTurnToAnglePIDConstants = new PIDConstants(12.0, 0.0, 0.05);
    public static final Constraints kTurnToAngleTolerances = new Constraints(0.017, 0.5); 

    ////////////////////////////////////////////
    /// -- NerdDrivetrain Swerve Requests -- ///
    ////////////////////////////////////////////

    /** Used for AutoBuilder configuration */
    public static final SwerveRequest.ApplyRobotSpeeds  kApplyRobotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
    /** Robot oriented controller */
    public static final SwerveRequest.RobotCentric      kRobotOrientedSwerveRequest = 
      new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.Position);

    /** Field oriented controller - use @see NerdDrivertrain#resetFieldOrientation() */
    public static final SwerveRequest.FieldCentric      kFieldOrientedSwerveRequest = 
      new SwerveRequest.FieldCentric()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.Position)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    /** Field oriented controller - use @see NerdDrivertrain#resetFieldOrientation() */
    public static final SwerveRequest.SwerveDriveBrake  kTowSwerveRequest = 
      new SwerveRequest.SwerveDriveBrake()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.Position);

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
      // Add field positions
      HUB_CENTER(4.626, 4.035, 0.0);
      
      public Pose2d blue, red; // meters and degrees
      FieldPositions(double _blueX, double _blueY, double _blueHeadingDegrees) {
        blue = new Pose2d(new Translation2d(_blueX, _blueY), new Rotation2d(Units.degreesToRadians(_blueHeadingDegrees)));
        red = FlippingUtil.flipFieldPose(blue);
      }

      public Pose2d get() {
        RobotContainer.refreshAlliance();
        return RobotContainer.IsRedSide() ? this.red : this.blue;
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

  public static final class LoggingConstants {
    public static final double LOGGING_INTERVAL = 0.02; // seconds

    public static final String kSubsystemTab = "SuperSystem/"; // ends with a /
    public static final String kSupersystemTab = "SuperSystem";
    public static final String kSwerveTab = "SwerveDrive";
    public static final String kAutosTab = "Autos";
  }

  public static final class VisionConstants {
    /** how many frames to skip in disabled, to prevent overheating */
    public static final int kDisabledThrottle = 100;

    public static enum Camera {
      // Example("limelight-ex", "10.6.87.XX:5802"),
      Front("limelight-fr", "10.6.87.17:5802"),
      Back("limelight-br", "10.6.87.15:5802");

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
        .withKD(0.0);

    private static final MotorOutputConfigs kMotorOutputConfigs =
      new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);

    private static final MotionMagicConfigs kMotionMagicConfigs = 
      new MotionMagicConfigs()
        .withMotionMagicAcceleration(36)
        .withMotionMagicCruiseVelocity(18);
    
    private static final CurrentLimitsConfigs kCurrentLimitsConfigs =
      new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withMotorOutput(kMotorOutputConfigs)
        .withMotionMagic(kMotionMagicConfigs)
        .withCurrentLimits(kCurrentLimitsConfigs);
  }

  public static final class IntakeRollerConstants {
    public static final int kMotor1ID = 15;

    private static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.0);

    private static final CurrentLimitsConfigs kCurrentLimitsConfigs = 
      new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withStatorCurrentLimitEnable(true);

    private static final FeedbackConfigs kFeedbackConfigs = 
      new FeedbackConfigs()
        .withRotorToSensorRatio(1.0)
      ;

    public static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withCurrentLimits(kCurrentLimitsConfigs)
        .withFeedback(kFeedbackConfigs);
    
  }
  
  public static final class IndexerConstants {
    public static final int kMotor1ID = 25;
    public static final int kMotor2ID = 24;

    public static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.0);

    public static final MotorOutputConfigs kMotorOutputConfigs =
      new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive);

    public static final CurrentLimitsConfigs kCurrentLimitsConfigs = 
      new CurrentLimitsConfigs()
        .withStatorCurrentLimit(50)
        .withStatorCurrentLimitEnable(true);
        
    public static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withMotorOutput(kMotorOutputConfigs)
        .withCurrentLimits(kCurrentLimitsConfigs);
  }
   
  public static final class ConveyorConstants {
    public static final int kMotor1ID = 26;

    private static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.0);

    public static final MotorOutputConfigs kMotorOutputConfigs =
      new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive);

    public static final CurrentLimitsConfigs kCurrentLimitsConfigs = 
      new CurrentLimitsConfigs()
        .withStatorCurrentLimit(30)
        .withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withMotorOutput(kMotorOutputConfigs);
  }
  
  public static final class ShooterConstants {
    public static final int kMotor1ID = 35;
    public static final int kMotor2ID = 36;
    public static final int kMotor3ID = 37;
    public static final int kMotor4ID = 38;

    private static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
        .withKP(0.15)
        .withKI(0.0)
        .withKD(0.0)
        .withKV(0.113)
        .withKS(0.2);
    
    private static final CurrentLimitsConfigs kCurrentLimitsConfigs = 
      new CurrentLimitsConfigs()
        .withStatorCurrentLimit(60)
        .withStatorCurrentLimitEnable(false);

    private static final MotionMagicConfigs kMotionMagicConfigs = 
      new MotionMagicConfigs()
        .withMotionMagicAcceleration(25);

    private static final MotorOutputConfigs kMotorOutputConfigs =
      new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive);

    private static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withCurrentLimits(kCurrentLimitsConfigs)
        .withMotionMagic(kMotionMagicConfigs)
        .withMotorOutput(kMotorOutputConfigs);

    // Regression of a*x^2 + b
    // Update at -- on -/--/2026
    public static final double kShootWithDistanceA = 0.87; // a
    public static final double kShootWithDistanceB = 31.60409; // b

    public static final double kLookAheadRingDriveFactor = 0.3; // use to tune the ring drive
    public static final double kLookAheadFactor = 1.35; // use to tune shoot on the move left and right
  }

  public static class LEDConstants {
    public static final int kCANdleID = 2;

    public static final CANdleConfiguration configuration = 
      new CANdleConfiguration()
        .withLED(
          new LEDConfigs()
            .withStripType(StripTypeValue.GRB)
            .withBrightnessScalar(0.5)
            .withLossOfSignalBehavior(LossOfSignalBehaviorValue.DisableLEDs)
        )
      ;

    public static class Colors {
      public static final double brightness = 0.2;
      public static final RGBWColor kRED           = new RGBWColor(255, 0, 0).scaleBrightness(brightness); 
      public static final RGBWColor kORANGE        = new RGBWColor(255, 80, 0).scaleBrightness(brightness); 
      public static final RGBWColor kYELLOW        = new RGBWColor(255, 255, 0).scaleBrightness(brightness); 
      public static final RGBWColor kGREEN         = new RGBWColor(0, 255, 0).scaleBrightness(brightness); 
      public static final RGBWColor kCYAN          = new RGBWColor(0, 255, 255).scaleBrightness(brightness); 
      public static final RGBWColor kBLUE          = new RGBWColor(0, 0, 255).scaleBrightness(brightness); 
      public static final RGBWColor kPURPLE        = new RGBWColor(255, 0, 255).scaleBrightness(brightness); 
      public static final RGBWColor kNERDHERD_BLUE = new RGBWColor(34, 105, 255).scaleBrightness(brightness); // #071635 as base, brightened fully
      public static final RGBWColor kBLACK         = new RGBWColor(0, 0, 0).scaleBrightness(brightness); // shows up as nothing
      public static final RGBWColor kWHITE         = new RGBWColor(255, 255,255).scaleBrightness(0.5).scaleBrightness(brightness); 
    }
    
    public enum LEDSegments {
      ALL(0, 69),
      CANDLE(0, 7),
      VERTICAL(8, 29),
      HORIZONTAL(30,69)
      ;
      
      // from bottom up/left to right
      public int start, end;
      LEDSegments(int startIndex, int endIndex) {
        this.start = startIndex;
        this.end = endIndex;
      }
    }

    public static class Animations {
      public static final LarsonAnimation kDisconnectedVertical = 
        new LarsonAnimation(LEDSegments.VERTICAL.start, LEDSegments.VERTICAL.end)
          .withSlot(0)
          .withColor(Colors.kNERDHERD_BLUE)
          .withSize(10)
          .withFrameRate(20)
        ;
      public static final LarsonAnimation kDisconnectedHorizontal =
        new LarsonAnimation(LEDSegments.HORIZONTAL.start, LEDSegments.HORIZONTAL.end)
          .withSlot(1)
          .withColor(Colors.kNERDHERD_BLUE)
          .withSize(25)
          .withFrameRate(15)
        ;
      // public static final RainbowAnimation kDisabled = 
      //   new RainbowAnimation(LEDSegments.HORIZONTAL.start, LEDSegments.HORIZONTAL.end)
      //     .withSlot(1)
      //     // .withColor(Colors.kGREEN)
      //     // .withSize(25)
      //     .withFrameRate(15)
      //     .withBrightness(Colors.brightness)
      //   ;
      public static final LarsonAnimation kDisabled = 
        new LarsonAnimation(LEDSegments.HORIZONTAL.start, LEDSegments.HORIZONTAL.end)
          .withSlot(1)
          .withColor(Colors.kGREEN)
          .withSize(25)
          .withFrameRate(15)
        ;
      public static final FireAnimation kAutonomousVertical = 
        new FireAnimation(LEDSegments.VERTICAL.start, LEDSegments.VERTICAL.end)
          .withSlot(0)
          .withCooling(0.2)
          .withFrameRate(20)
        ;
      public static final SolidColor kDefaultHorizontal = 
        new SolidColor(LEDSegments.HORIZONTAL.start, LEDSegments.HORIZONTAL.end)
          .withColor(Colors.kNERDHERD_BLUE)
        ;
      public static final SolidColor kTeleopVertical =
        new SolidColor(LEDSegments.VERTICAL.start, LEDSegments.VERTICAL.end)
          .withColor(Colors.kNERDHERD_BLUE)  
        ;
      public static final SolidColor kCountdown = 
        new SolidColor(LEDSegments.HORIZONTAL.start, LEDSegments.HORIZONTAL.end)
          .withColor(Colors.kRED)
        ;
      public static final SolidColor kShooterRamp = 
        new SolidColor(LEDSegments.VERTICAL.start, LEDSegments.VERTICAL.end)
          .withColor(Colors.kORANGE)
        ;
    }
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
        SubsystemMode.VOLTAGE, 
        0.0,
        useIntakeSlapdown)
      .configureMotors(IntakeSlapdownConstants.kSubsystemConfiguration)
      .logTorqueCurrent();
    
    public static final boolean useIntakeRoller = true;
    public static final TemplateSubsystem intakeRoller = (!USE_SUBSYSTEMS) ? null :
    new TemplateSubsystem(
        "Intake Roller", 
        IntakeRollerConstants.kMotor1ID, 
        SubsystemMode.VOLTAGE, 
        0.0,
        useIntakeRoller)
      .configureMotors(IntakeRollerConstants.kSubsystemConfiguration);
    
    public static final boolean useConveyor = true;
    public static final TemplateSubsystem conveyor = (!USE_SUBSYSTEMS) ? null :
    new TemplateSubsystem(
        "Conveyor", 
        ConveyorConstants.kMotor1ID, 
        SubsystemMode.VOLTAGE, 
        0.0,
        useConveyor)
      .configureMotors(ConveyorConstants.kSubsystemConfiguration);    
    
    public static final boolean useIndexer = true;
    public static final TemplateSubsystem indexer = (!USE_SUBSYSTEMS) ? null :
    new TemplateSubsystem(
        "Indexer", 
        IndexerConstants.kMotor1ID, 
        SubsystemMode.VOLTAGE, 
        0.0,
        useIndexer)
      .addMotor(IndexerConstants.kMotor2ID, MotorAlignmentValue.Opposed)
      .configureMotors(IndexerConstants.kSubsystemConfiguration);
    
    public static final boolean useShooter = true;
    public static final TemplateSubsystem shooter = (!USE_SUBSYSTEMS) ? null :
    new TemplateSubsystem(
        "Shooter", 
        ShooterConstants.kMotor1ID,
        SubsystemMode.PROFILED_VELOCITY, 
        0.0,
        useShooter)
      .addMotor(ShooterConstants.kMotor2ID, MotorAlignmentValue.Opposed)
      .addMotor(ShooterConstants.kMotor3ID, MotorAlignmentValue.Opposed)
      .addMotor(ShooterConstants.kMotor4ID, MotorAlignmentValue.Aligned)
      .configureMotors(ShooterConstants.kSubsystemConfiguration)
      .logTorqueCurrent();
    
    public static final boolean useLEDs = false;
    public static final LED leds = (!useLEDs) ? null : 
      new LED(
        LEDConstants.kCANdleID, 
        LEDConstants.configuration
      );
    /**
     * literally just so the class actually loads,
     * thanks java lazy loading
     */
    public static void init() {} 
  }
}
