// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.util.NerdyMath;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RingDriveCommand extends Command {
  private final NerdDrivetrain swerveDrive;
  private double targetX, targetY, targetR;

  public RingDriveCommand(NerdDrivetrain swerveDrive) {
    this.swerveDrive = swerveDrive;

    addRequirements(this.swerveDrive);
  }

  @Override
  public void initialize() {
    targetX = swerveDrive.getPose().getX();
    targetY = swerveDrive.getPose().getY();
    targetR = NerdyMath.angleToPose(swerveDrive.getPose(), FieldPositions.HUB_CENTER.blue);
    swerveDrive.resetTargetDrive();
  }

  @Override
  public void execute() {
    swerveDrive.driveToTarget(new Pose2d(targetX, targetY, Rotation2d.fromRadians(targetR)));
  }
}
