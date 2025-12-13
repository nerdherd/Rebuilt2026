// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class NerdDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

  public NerdDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
    super(drivetrainConstants, modules);
    
    RobotConfig robotConfig = null;
    try {
        robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
        e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> setChassisSpeeds(speeds),
        new PPHolonomicDriveController(
            kPPTranslationPIDConstants, 
            kPPRotationPIDConstants), 
            // kPPMaxVelocity,
            // kTrackWidth,
            // new ReplanningConfig()), 
        robotConfig,
        () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
    );
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
