// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.RingDriveConstants;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.util.NerdyMath;
import frc.robot.util.filters.DeadbandFilter;
import frc.robot.util.filters.Filter;

public class RingDriveCommand extends Command {
  private final NerdDrivetrain swerveDrive;
  private double targetTheta, targetD;
  private final Supplier<Double> xInput, yInput;
  private Filter xFilter, yFilter;
  private Pose2d center;

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

    this.xFilter = new DeadbandFilter(
      ControllerConstants.kDeadband
      );
    this.yFilter = new DeadbandFilter(
      ControllerConstants.kDeadband
      );

    center = Pose2d.kZero;
    addRequirements(this.swerveDrive);
  }

  @Override
  public void initialize() {
    /*
    RobotContainer.refreshAlliance();
    if (RobotContainer.IsRedSide()) center = FieldPositions.HUB_CENTER.red;
    else center = FieldPositions.HUB_CENTER.blue;
     */
    
    targetD = RingDriveConstants.kInitialDistance;
    targetTheta = NerdyMath.angleToPose(center, swerveDrive.getPose());
    swerveDrive.resetTargetDrive();
  }

  @Override
  public void execute() {
    double xIn = xInput.get();
    double yIn = yInput.get();
    double filteredXSpeed = xFilter.calculate(xIn); // TODO reconsider filters
    double filteredYSpeed = yFilter.calculate(yIn);

    double ySpeed = filteredXSpeed*RingDriveConstants.kDriveVelocity;
    double xSpeed = filteredYSpeed*(RingDriveConstants.kDriveVelocity / targetD);

    targetD -= ySpeed / 50.0;
    targetD = NerdyMath.clamp(targetD, RingDriveConstants.kMinimumDistance, RingDriveConstants.kMaximumDistance);

    targetTheta += xSpeed / 50.0;

    swerveDrive.driveToTarget(new Pose2d(targetD*Math.cos(targetTheta) + center.getX(), targetD*Math.sin(targetTheta) + center.getY(), Rotation2d.fromRadians(targetTheta + Math.PI)));
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.driveFieldOriented(0.0, 0.0, 0.0);
  }
}
