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
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class NerdDrivetrain extends TunerSwerveDrivetrain implements Subsystem, Reportable {
    public final Field2d field;
    public boolean useMegaTag2 = true;

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
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        field = new Field2d();

        setVision(false);
    }
    
    
    @Override
    public void periodic() {
        field.setRobotPose(getPose());

        if (USE_VISION) {
            // visionUpdate(Camera.Example); TODO add cameras separately
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
     * TODO find out if getPose().getRotation() is the same as getAbsoluteHeadingDegrees
     * drive to a target in blue oriented space, must be called continously at 50 Hz
     * call {@link #resetTargetDrive()} when starting
     * @param target - point in blue oriented space
     */
    public void driveToTarget(Pose2d target) {
        // since the outputs are also capped, there is a limit to the influence of one axis on the direction
        // this can lead to larger angles, maybe saving on necessary precision?
        double x = kTargetDriveController.calculate("x", getPose().getX(), target.getX());
        double y = kTargetDriveController.calculate("y", getPose().getY(), target.getY());
        double l = Math.sqrt(x*x+y*y);
        // clamp the velocity
        x *= Math.min(1.0, kTargetDriveMaxLateralVelocity / l);
        y *= Math.min(1.0, kTargetDriveMaxLateralVelocity / l);
        driveFieldOriented(
            x,
            y,
            kTargetDriveController.calculate("r", getPose().getRotation().getRadians(), target.getRotation().getRadians())
        );
    }

    /**
     * resets the target drive controller for {@link #driveToTarget(Pose2d)}
     */
    public void resetTargetDrive() {
        kTargetDriveController.reset(
            getPose().getX(), 
            getPose().getY(), 
            getPose().getRotation().getRadians()
        );
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

    /** 
     * get absolute heading in degrees, from blue alliance rotation
     * @see {@link #resetRotation(Rotation2d)}
     */
    public double getAbsoluteHeadingDegrees() {
        return getPigeon2().getRotation2d().getDegrees();
    }

    /**
     * get heading relative to what the operator sees
     * @see {@link #zeroFieldOrientation()} for resetting to zero
     * @see {@link #seedFieldCentric()} for the same thing as above
     */
    public double getOperatorHeadingDegrees() {
        return getOperatorForwardDirection().getDegrees();
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

    // ----------------------------------------- Vision Functions ----------------------------------------- //

    /**
     * activates or deactivates vision by setting the pipeline either to 1 for active or 0 for inactive
     * and by adjusting throttle, see {@link LimelightHelpers#SetThrottle(String, int)}
     * @param activate whether to activate or deactivate
     */
    public void setVision(boolean activate) {
        for (Camera camera : Camera.values()) {
            LimelightHelpers.setPipelineIndex(camera.name, (activate) ? 1 : 0);
            LimelightHelpers.SetThrottle(camera.name, (activate) ? 0 : Constants.VisionConstants.kDisabledThrottle);
        }
    }

    public void visionUpdate(Camera limelight) {
        if (!useMegaTag2) {
            // --------- MT1 --------- //
            useMegaTag2 = true; // TODO megatag1 gyro initialization
        }
        else {
            // --------- MT2 --------- //
            double yaw = getAbsoluteHeadingDegrees();
            LimelightHelpers.SetRobotOrientation(limelight.name, yaw, 0, 0, 0, 0, 0);
            PoseEstimate mt = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name);
            if (mt == null || Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720 || mt.tagCount == 0) return;
            setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999)); // TODO consider other stddevs
            addVisionMeasurement(mt.pose, mt.timestampSeconds);
        }
    }

    // ----------------------------------------- Gyro Functions ----------------------------------------- //
    
    /** 
     * use with bindings to reset the field oriented control 
     * @see {@link #seedFieldCentric}
     * @see {@link #setOperatorPerspectiveForward} also for more custom setting
     */
    public void zeroFieldOrientation() {
        seedFieldCentric();
    }

    // ----------------------------------------- Logging Functions ----------------------------------------- //

    @Override
    public void initializeLogging() {
        ShuffleboardTab tab = Shuffleboard.getTab("NerdDrivetrain");
        tab.add("Robot Field", field).withSize(6,3);

        ///////////
        /// ALL ///
        ///////////
        for (Camera camera : Camera.values())
            Reportable.addCamera(tab, camera.name, camera.name, "http://" + camera.ip, LOG_LEVEL.ALL);
        if (Constants.ROBOT_LOG_LEVEL.level == LOG_LEVEL.ALL.level) {
            Field2d positionField = new Field2d();
            for (FieldPositions position : FieldPositions.values()) {
                FieldObject2d blue = positionField.getObject(position.name() + "-blue");
                blue.setPose(position.blue);
                FieldObject2d red  = positionField.getObject(position.name() + "-red");
                red.setPose(position.red);
            }
            tab.add("Position Field", positionField).withSize(6,3);
        }
        
        //////////////
        /// MEDIUM ///
        //////////////
        Reportable.addNumber(tab, "heading", this::getAbsoluteHeadingDegrees, LOG_LEVEL.MEDIUM);
        
        //////////////
        /// MINIMAL //
        //////////////
        Reportable.addString(tab, "temperatures", this::pollTemperatures, LOG_LEVEL.MINIMAL); // maybe better on medium
        // TODO poll voltages somehow, maybe just log all of them
    }

}
