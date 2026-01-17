// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RingDriveConstants;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.util.NerdyMath;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RingDriveCommand extends Command {
  private final NerdDrivetrain swerveDrive;
  private double targetTheta, targetD;
  private final Supplier<Double> xInput, yInput;

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
    double ySpeed = yInput.get()*RingDriveConstants.kDriveVelocity / 50.0;
    double xSpeed = xInput.get()*((RingDriveConstants.kDriveVelocity / targetD) / 50.0);

    targetD -= ySpeed;
    targetD = Math.max(targetD, RingDriveConstants.kMinimumDistance);

    targetTheta += xSpeed;

    swerveDrive.driveToTarget(new Pose2d(targetD*Math.cos(targetTheta), targetD*Math.sin(targetTheta), Rotation2d.fromRadians(targetTheta + Math.PI)));
  }
}
