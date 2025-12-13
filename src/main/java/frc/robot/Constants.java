// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.subsystems.template.TemplateSubsystem.LOG_LEVEL;

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

  public static LOG_LEVEL LOGGING_LEVEL = LOG_LEVEL.MINIMAL;
  public static boolean USE_SUBSYSTEMS = true;

  public static class ControllerConstants {
    public static final double kDeadband = 0.05;
    public static final double kRotationDeadband = 0.1;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  
  public static final class ModuleConstants {

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / 21.428; // 150 : 7 : 1 MK4i
    // public static final double kDriveDistanceLoss = 0.95; // from measuring IRL
    public static final double kDriveDistanceLoss = 1; // from measuring IRL
    public static final double kMetersPerRevolution = kWheelDiameterMeters * Math.PI * kDriveDistanceLoss;
    public static final double kDriveTicksToMeters = (1 / 2048.0) * kMetersPerRevolution; 
    public static final double kAbsoluteTurningTicksToRad = (1.0 / 4096.0) * 2 * Math.PI;
    public static final double kIntegratedTurningTicksToRad = (1.0 / 2048.0) * 2 * Math.PI;
    public static final double kDriveTicksPer100MsToMetersPerSec = kDriveTicksToMeters * 10;
    public static final double kAbsoluteTurningTicksPer100MsToRadPerSec = kAbsoluteTurningTicksToRad * 10;
    public static final double kIntegratedTurningTicksPer100MsToRadPerSec = kIntegratedTurningTicksToRad * 10;

    public static final double kDriveMotorDeadband = 0.02;
    public static final double kTurnMotorDeadband = 0.001;

    public static final double kPTurning = 0.55; // 0.55
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.0015;//0.02 
    public static final double kFTurning = 0.015;//0.015

    public static final double kPDrive = 0.13; // 0.6
    public static final double kIDrive = 0;
    public static final double kDDrive = 0; 
    public static final double kVDrive = 0.0469; 

    public static final String kCANivoreName = "CANivore";//"CANivore";

  } 

  public static final class SwerveDriveConstants {

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
    
    public static final int kFRDriveID = 11;
    public static final int kFLDriveID = 21;
    public static final int kBLDriveID = 31;
    public static final int kBRDriveID = 41;

    public static final int kFRTurningID = 12;
    public static final int kFLTurningID = 22;
    public static final int kBLTurningID = 32;
    public static final int kBRTurningID = 42;

    public static final boolean kFRTurningReversed = true;
    public static final boolean kFLTurningReversed = true; 
    public static final boolean kBLTurningReversed = true; 
    public static final boolean kBRTurningReversed = true; 

    public static final boolean kFRDriveReversed = false;
    public static final boolean kFLDriveReversed = false;     
    public static final boolean kBLDriveReversed = false;      
    public static final boolean kBRDriveReversed = false;

    public static final class CANCoderConstants {
      public static final int kFRCANCoderID = 14;
      public static final int kFLCANCoderID = 24;
      public static final int kBLCANCoderID = 34;
      public static final int kBRCANCoderID = 44;

      public static final boolean kFRCANCoderReversed = false;    
      public static final boolean kFLCANCoderReversed = false;      
      public static final boolean kBLCANCoderReversed = false;       
      public static final boolean kBRCANCoderReversed = false; 
    }

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

    public static final double kGravityMPS = 9.80665; 

    public static final double kTurnToAnglePositionToleranceAngle = 0.5;
    public static final double kTurnToAngleVelocityToleranceAnglesPerSec = 1;

    public static enum FieldPositions {
      ;//TODO Add field positions
      
      public Pose2d pos;
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
