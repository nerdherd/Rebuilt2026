// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RingDriveConstants;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.util.NerdyMath;

public class RingDriveCommand extends Command {
  private final NerdDrivetrain swerveDrive;
  private double targetTheta, targetD;
  private final Supplier<Double> xInput, yInput;

  /**
   * Creates a ring drive command that rotates around a given point
   * @param swerveDrive
   * @param xInput - CCW+
   * @param yInput - Inwards+
   */
  public RingDriveCommand(NerdDrivetrain swerveDrive, Supplier<Double> xInput, Supplier<Double> yInput) {
    this.swerveDrive = swerveDrive;
    this.xInput = xInput;
    this.yInput = yInput;

    addRequirements(this.swerveDrive);
  }

  @Override
  public void initialize() {
    targetD = RingDriveConstants.kInitialDistance;
    targetTheta = NerdyMath.angleToPose(Pose2d.kZero, swerveDrive.getPose());
    swerveDrive.resetTargetDrive();
  }

  @Override
  public void execute() {
    double ySpeed = yInput.get()*RingDriveConstants.kDriveVelocity;
    double xSpeed = xInput.get()*(RingDriveConstants.kDriveVelocity / targetD);

    targetD -= ySpeed / 50.0;
    targetD = NerdyMath.clamp(targetD, RingDriveConstants.kMinimumDistance, RingDriveConstants.kMaximumDistance);

    targetTheta += xSpeed / 50.0;

    swerveDrive.driveToTarget(new Pose2d(targetD*Math.cos(targetTheta) + 0.0, targetD*Math.sin(targetTheta) + 0.0, Rotation2d.fromRadians(targetTheta + Math.PI)));
  }
}
