// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.USE_VISION;
import static frc.robot.Constants.PathPlannerConstants.kPPRotationPIDConstants;
import static frc.robot.Constants.PathPlannerConstants.kPPTranslationPIDConstants;
import static frc.robot.Constants.SwerveDriveConstants.kApplyRobotSpeedsRequest;
import static frc.robot.Constants.SwerveDriveConstants.kFieldOrientedSwerveRequest;
import static frc.robot.Constants.SwerveDriveConstants.kRobotOrientedSwerveRequest;
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveController;
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveMaxLateralVelocity;
import static frc.robot.Constants.SwerveDriveConstants.kTowSwerveRequest;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class NerdDrivetrain extends TunerSwerveDrivetrain implements Subsystem, Reportable {
    public final Field2d field;
    public boolean useMegaTag2 = false;

    public NerdDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        RobotConfig robotConfig = null;
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getChassisSpeeds,
            (speeds, feedforwards) -> setControl(
                kApplyRobotSpeedsRequest.withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
            new PPHolonomicDriveController(
                kPPTranslationPIDConstants, 
                kPPRotationPIDConstants),  
            robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? (alliance.get() == DriverStation.Alliance.Red) : false;
            },
            this
        );

        field = new Field2d();

        setVision(USE_VISION);
    }
    
    
    @Override
    public void periodic() {
        field.setRobotPose(getPose());

        if (USE_VISION) {
            // visionUpdate(Camera.Example);
            visionUpdate(Camera.Charlie);
        }
    }

    // ----------------------------------------- Drive Functions ----------------------------------------- //

    /**
     * drive in field oriented
     * @param xSpeed - how fast in the +x direction in m/s
     * @param ySpeed - how fast in the +y direction in m/s
     * @param rSpeed - how fast in the CCW direction in rad/s
     */
    public void driveFieldOriented(double xSpeed, double ySpeed, double rSpeed) {
        setControl(kFieldOrientedSwerveRequest
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rSpeed)
        );
    }

    /**
     * drive in robot oriented
     * @param xSpeed - how fast in the +x direction (forward) in m/s
     * @param ySpeed - how fast in the +y direction (left) in m/s
     * @param rSpeed - how fast in the CCW direction in rad/s
     */
    public void driveRobotOriented(double xSpeed, double ySpeed, double rSpeed) {
        setControl(kRobotOrientedSwerveRequest
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rSpeed)
        );
    }

    /**
     * drive to a target in blue oriented space, must be called continously at 50 Hz
     * call {@link #resetTargetDrive()} when starting
     * @param target - point in blue oriented space
     */
    public void driveToTarget(Pose2d target) {
        // since the outputs are also capped, there is a limit to the influence of one axis on the direction
        // this can lead to larger angles, maybe saving on necessary precision?
        // DriverStation.reportWarning(controller.getConstraints().maxVelocity + " " + controller.getConstraints().maxAcceleration, false);
        double x = kTargetDriveController.calculate("x", getPose().getX(), target.getX());
        double y = kTargetDriveController.calculate("y", getPose().getY(), target.getY());
        double r = kTargetDriveController.calculate("r", getSwerveHeadingRadians(), MathUtil.inputModulus(target.getRotation().getRadians(), -Math.PI, Math.PI));
        double l = Math.sqrt(x*x+y*y);
        // clamp the velocity
        x *= Math.min(1.0, kTargetDriveMaxLateralVelocity / l);
        y *= Math.min(1.0, kTargetDriveMaxLateralVelocity / l);
        if (kTargetDriveController.atSetpoint("x")) x = 0.0;
        if (kTargetDriveController.atSetpoint("y")) y = 0.0;
        if (kTargetDriveController.atSetpoint("r")) r = 0.0;
        // DriverStation.reportWarning(x + " " + y + " " + r, false);

        driveFieldOriented(x, y, r);
    }

    /**
     * resets the target drive controller for {@link #driveToTarget(Pose2d)}
     */
    public void resetTargetDrive() {
        kTargetDriveController.reset("x", getPose().getX(), getFieldOrientedSpeeds().vxMetersPerSecond * 0.1);
        kTargetDriveController.reset("y", getPose().getY(), getFieldOrientedSpeeds().vyMetersPerSecond * 0.1);
        kTargetDriveController.reset("r", getSwerveHeadingRadians(), getFieldOrientedSpeeds().omegaRadiansPerSecond * 0.1);
    }

    // ----------------------------------------- Helper Functions ----------------------------------------- //

    public void stop() {
        setControl(kTowSwerveRequest);
    }

    /** gets the Pose2d from odometry */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /** gets the ChassisSpeeds from odometry */
    public ChassisSpeeds getChassisSpeeds() {
        return getState().Speeds;
    }

    public ChassisSpeeds getFieldOrientedSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), Rotation2d.fromDegrees(getSwerveHeadingDegrees()));
    }

    public void setBrake(boolean brake) {
        configNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private final double deviceTempThreshold = 50;
    /** @return "it's chill" unless the temperature of any motor is above the threshold, reports id and type */
    private String pollTemperatures() {
        String output = "";
        for (int i = 0; i < 4; i++) {
            double driveTemp = getModule(i).getDriveMotor().getDeviceTemp().getValueAsDouble();
            double steerTemp = getModule(i).getSteerMotor().getDeviceTemp().getValueAsDouble();
            if (driveTemp >= deviceTempThreshold) output += "Drive " + i + ", Temp: " + driveTemp;
            if (steerTemp >= deviceTempThreshold) output += "Steer " + i + ", Temp: " + steerTemp;
            
        }
        if (!output.equals("")) return output;
        return "it's chill";
    }

    private double pollStatorCurrentSum() {
        double sum = 0;
        for (int i = 0; i < 4; i++) {
            sum += getModule(i).getDriveMotor().getTorqueCurrent().getValueAsDouble();
            sum += getModule(i).getSteerMotor().getTorqueCurrent().getValueAsDouble();
        }
        return sum;
    }

    // ----------------------------------------- Vision Functions ----------------------------------------- //

    /**
     * activates or deactivates vision by setting the pipeline either to 0 for active or 1 for inactive
     * and by adjusting throttle, see {@link LimelightHelpers#SetThrottle(String, int)}
     * @param activate whether to activate or deactivate
     */
    public void setVision(boolean activate) {
        for (Camera camera : Camera.values()) {
            LimelightHelpers.setPipelineIndex(camera.name, (activate) ? 0 : 1);
            LimelightHelpers.SetThrottle(camera.name, (activate) ? 0 : Constants.VisionConstants.kDisabledThrottle);
        }
    }
    
    /**
     * temporarily switch to megatag1 to update robot field heading/pose
     * @param delay in seconds until switching back to megatag2
     */
    public Command resetPoseWithAprilTags(double delay) {
        return Commands.sequence(
            Commands.runOnce(() -> useMegaTag2 = false),
            Commands.waitSeconds(delay),
            Commands.runOnce(() -> useMegaTag2 = true)
      );
    }

    public void visionUpdate(Camera limelight) {
        if (LimelightHelpers.getCurrentPipelineIndex(limelight.name) != 0) return;
        if (!useMegaTag2) {
            // --------- MT1 --------- //
            PoseEstimate mt = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name);
            if (mt == null || Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720 || mt.tagCount == 0) return;
            resetRotation(mt.pose.getRotation());
        }
        else {
            // --------- MT2 --------- //
            double yaw = getSwerveHeadingDegrees();
            LimelightHelpers.SetRobotOrientation(limelight.name, yaw, 0, 0, 0, 0, 0);
            PoseEstimate mt = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name);
            if (mt == null || Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720 || mt.tagCount == 0) return;
            setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999)); // TODO consider other stddevs
            addVisionMeasurement(mt.pose, Utils.getCurrentTimeSeconds());
        }
    }

    // ----------------------------------------- Gyro Functions ----------------------------------------- //

    /** 
     * set the operator heading to forward based on alliance field forward
     * @see {@link #setOperatorPerspectiveForward} also for more custom setting
     */
    public void setDriverHeadingForward() {
        setOperatorPerspectiveForward(RobotContainer.IsRedSide() ? Rotation2d.k180deg : Rotation2d.kZero);
    }

    /** 
     * set the operator heading to forward based on robot
     * @see {@link #setOperatorPerspectiveForward} also for more custom setting
     */
    public void setRobotHeadingForward() {
        setOperatorPerspectiveForward(getPose().getRotation());
    }
    
    /**
     * get heading relative to what the operator sees in degrees
     * @see {@link #setDriverHeadingForward()} for resetting to zero
     */
    public double getDriverHeadingDegrees() {
        return getOperatorForwardDirection().getDegrees() + getSwerveHeadingDegrees();
    }

    /**
     * get heading relative to what the operator sees in radians
     * @see {@link #setDriverHeadingForward()} for resetting to zero
     */
    public double getDriverHeadingRadians() {
        return getOperatorForwardDirection().getRadians() + getSwerveHeadingRadians();
    }

    /** 
     * get absolute heading in degrees, from blue alliance orientation
     * @see {@link #resetRotation(Rotation2d)}
     */
    public double getSwerveHeadingDegrees() {
        return MathUtil.inputModulus(getPose().getRotation().getDegrees(), -180, 180);
    }

    /** 
     * get absolute heading in radians, from blue alliance orientation
     * @see {@link #resetRotation(Rotation2d)}
     */
    public double getSwerveHeadingRadians() {
        return MathUtil.inputModulus(getPose().getRotation().getRadians(), -Math.PI, Math.PI);
    }

    // ----------------------------------------- Logging Functions ----------------------------------------- //

    @Override
    public void initializeLogging() {
        String tab = "NerdDrivetrain";
        NerdLog.logData(tab, "Robot Field", () -> field, LOG_LEVEL.MINIMAL);

        ///////////
        /// ALL ///
        ///////////
        if (Constants.ROBOT_LOG_LEVEL == LOG_LEVEL.ALL) {
            Field2d positionField = new Field2d();
            for (FieldPositions position : FieldPositions.values()) {
                FieldObject2d blue = positionField.getObject(position.name() + "-blue");
                blue.setPose(position.blue);
                FieldObject2d red  = positionField.getObject(position.name() + "-red");
                red.setPose(position.red);
            }
            NerdLog.logData("NerdDrivetrain", "Position Field", () -> positionField, LOG_LEVEL.ALL);
        }

        NerdLog.logNumber(tab, "field chassis speeds x", () -> getFieldOrientedSpeeds().vxMetersPerSecond, "m/s", LOG_LEVEL.ALL);
        NerdLog.logNumber(tab, "field chassis speeds y", () -> getFieldOrientedSpeeds().vyMetersPerSecond, "m/s", LOG_LEVEL.ALL);
        NerdLog.logNumber(tab, "field chassis speeds r", () -> getFieldOrientedSpeeds().omegaRadiansPerSecond, "rad/s", LOG_LEVEL.ALL);
        
        //////////////
        /// MEDIUM ///
        //////////////
        NerdLog.logNumber(tab, "swerve heading", this::getSwerveHeadingDegrees, "deg", LOG_LEVEL.MEDIUM);
        NerdLog.logNumber(tab, "driver heading", this::getDriverHeadingDegrees, "deg", LOG_LEVEL.MEDIUM);
        
        //////////////
        /// MINIMAL //
        //////////////
        NerdLog.logString(tab, "temperatures", this::pollTemperatures, LOG_LEVEL.MINIMAL); // maybe better on medium
        NerdLog.logNumber(tab, "stator current sum", this::pollStatorCurrentSum, "A", LOG_LEVEL.MINIMAL);
    }

}
