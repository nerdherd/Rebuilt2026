// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.SuperSystem;

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
  public static LOG_LEVEL ROBOT_LOG_LEVEL = LOG_LEVEL.MINIMAL;
  /** 
   * (hopefully) controls whether subsystem objects are used, swerve and others not counted
   * @see {@link frc.robot.subsystems.template.TemplateSubsystem TemplateSubsystem} 
   * @see {@link frc.robot.subsystems.SuperSystem SuperSystem}
   */
  public static boolean USE_SUBSYSTEMS = true;
  /**
   * controls whether vision should be initialized
   */
  public static boolean USE_VISION = false; // TODO

  public static class ControllerConstants {
    public static final double kDeadband = 0.05;
    public static final double kRotationDeadband = 0.1;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  
  public static final class SwerveDriveConstants {
    public static final String kCANivoreName = "CANivore";//"CANivore";

    public static final double kVisionSTDx = 0.7; //0.9
    public static final double kVisionSTDy = 0.7; //0.9s
    public static final double kVisionSTDtheta = 1000; //Old: 69696969
    public static final Matrix<N3, N1> kBaseVisionPoseSTD = MatBuilder.fill(
                                                              Nat.N3(), Nat.N1(), 
                                                              kVisionSTDx,
                                                              kVisionSTDy,
                                                              kVisionSTDtheta);
    // VecBuilder.fill(kVisionSTDx, kVisionSTDy, kVisionSTDtheta);
    public static final double kPThetaAuto = 0.3;
    public static final double kIThetaAuto = 0;
    public static final double kDThetaAuto = 0.01;

    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(23.5); // 24.125
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(23.5); // 24.125

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;    
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleMaxAcceleration = 5;
    // THIS CONSTANT HAS TO BE NEGATIVE OTHERWISE THE ROBOT WILL CRASH
    // TODO: Change deceleration with driver feedback, only in small increments (<= -2 is dangerous)
    public static final double kTeleMaxDeceleration = -5; // Russell says he likes 2.5 from sims, but keep at 3 until tested on real robot 

    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
      kPhysicalMaxAngularSpeedRadiansPerSecond * 0.75;
    public static final double kTurnToAngleMaxAngularSpeedRadiansPerSecond 
      = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTurnToBigAngleMaxAngularSpeedRadiansPerSecond = 1.5 * Math.PI;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double kMinimumMotorOutput = 0.05; // Minimum percent output on the falcons
    
    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;

    public static final SwerveModuleState[] towModuleStates = 
    new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(135))
    };

    /** Used for AutoBuilder configuration */
    public static final SwerveRequest.ApplyRobotSpeeds  kApplyRobotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
    /** Robot oriented controller */
    public static final SwerveRequest.RobotCentric      kRobotOrientedSwerveRequest = new SwerveRequest.RobotCentric()
                                                                                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                                                                                        .withSteerRequestType(SteerRequestType.Position);
    /** Field oriented controller - use @see NerdDrivertrain#resetFieldOrientation() */
    public static final SwerveRequest.FieldCentric      kFieldOrientedSwerveRequest = new SwerveRequest.FieldCentric()
                                                                                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                                                                                        .withSteerRequestType(SteerRequestType.Position);
    /** Field oriented controller - use @see NerdDrivertrain#resetFieldOrientation() */
    public static final SwerveRequest.SwerveDriveBrake  kTowSwerveRequest = new SwerveRequest.SwerveDriveBrake()
                                                                                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                                                                                        .withSteerRequestType(SteerRequestType.Position);


    public static final double kGravityMPS = 9.80665; 

    public static final double kTurnToAnglePositionToleranceAngle = 0.5;
    public static final double kTurnToAngleVelocityToleranceAnglesPerSec = 1;

    public static enum FieldPositions {
      ;//TODO Add field positions
      
      public Pose2d pos; // meters and degrees
      FieldPositions(double _x, double _y, double _headingDegrees) {
        pos = new Pose2d(new Translation2d(_x, _y), new Rotation2d(Units.degreesToRadians(_headingDegrees)));
      }

    }
  }

  public static final class PathPlannerConstants {

    public static final double kPPMaxVelocity = 2.0;
    public static final double kPPMaxAcceleration = 2.0;
    public static final double kPPMaxAngularVelocity = Math.PI * 2;
    public static final double kPPMaxAngularAcceleration = Math.PI * 2;

    public static final double kPP_P = 5.0; //6
    public static final double kPP_I = 0.0;
    public static final double kPP_D = 0.0;

    public static final PIDConstants kPPTranslationPIDConstants = new PIDConstants(kPP_P, kPP_I, kPP_D);

    public static final double kPP_ThetaP = 4.0; //3
    public static final double kPP_ThetaI = 0;
    public static final double kPP_ThetaD = 0.1;

    public static final PIDConstants kPPRotationPIDConstants = new PIDConstants(kPP_ThetaP, kPP_ThetaI, kPP_ThetaD);

    public static final boolean kUseAllianceColor = true;

  }

  public static final class VisionConstants {
    // public static final String kLimelightName = "limelight-uhhhhh";
    // public static final String kLimelightIP = "10.6.87.67:5802";
  }

  public static class LEDConstants {

    public static final int CANdleID = 0; // TODO change later
    public static final int CANdleLength = 8; // TODO change later

    public static class Colors {
      public static final Color BLACK         = new Color(0.0, 0.0, 0.0); // shows up as nothing
      public static final Color WHITE         = new Color(1.0,1.0,1.0); 
      public static final Color RED           = new Color(1.0, 0.0, 0.0); 
      public static final Color GREEN         = new Color(0.0, 1.0, 0.0); 
      public static final Color BLUE          = new Color(0.0, 0.0, 1.0); 
      public static final Color NERDHERD_BLUE = new Color(0.132, 0.415, 1.0); // #071635 as base, brightened fully
    }
    
    public enum LEDStrips {
      ALL(0, CANdleLength),
      CANDLE(0,8),
      ;

      public int index, count;
      LEDStrips(int _index, int _count) {
        this.index = _index;
        this.count = _count;
      }
    }

  }

  // ************************************** SUBSYSTEM CONSTANTS *************************************** //

  // Template Copy and Paste
  // public static final class SubsystemConstants {
  //   public static final int kMotor1ID = ;
  //   public static final int kMotor2ID = ;

  //   public static final Slot0Configs kSlot0Configs = 
  //     new Slot0Configs()
  //       .withKP(0.0)
  //     ;

  //   public static final TalonFXConfiguration kSubsystemConfiguartion = 
  //     new TalonFXConfiguration()
  //     .withSlot0(kSlot0Configs)
  //     ;
  // }
  
  public static final class SuperSystemConstants {
    //TODO DO
  }
}
