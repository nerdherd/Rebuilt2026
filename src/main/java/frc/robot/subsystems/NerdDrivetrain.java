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
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveDistancePID;
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveMaxAngularVelocity;
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveMaxVelocity;
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveRotationPID;
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveRotationalPosTolerance;
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveRotationalVelTolerance;
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveTranslationalPosTolerance;
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveTranslationalVelTolerance;
import static frc.robot.Constants.SwerveDriveConstants.kTowSwerveRequest;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.NerdyMath;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class NerdDrivetrain extends TunerSwerveDrivetrain implements Subsystem, Reportable {
    public final Field2d field;
    public boolean useMegaTag2 = true;

    /** controls distance {@link #driveToTarget(Pose2d)} */
    private final PIDController distanceController;
    /** controls rotation {@link #driveToTarget(Pose2d)} */
    private final PIDController rotationController;

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

        distanceController = new PIDController(kTargetDriveDistancePID.kP, kTargetDriveDistancePID.kI, kTargetDriveDistancePID.kD);
        rotationController = new PIDController(kTargetDriveRotationPID.kP, kTargetDriveRotationPID.kI, kTargetDriveRotationPID.kD);
        distanceController.setTolerance(kTargetDriveTranslationalPosTolerance, kTargetDriveTranslationalVelTolerance);
        rotationController.setTolerance(kTargetDriveRotationalPosTolerance, kTargetDriveRotationalVelTolerance);
        rotationController.enableContinuousInput(Math.PI, -Math.PI);
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
     * TODO move to another class and find out if getPose().getRotation() is the same as getAbsoluteHeadingDegrees
     * drive to a target in blue oriented space
     * @param target - point in blue oriented space
     */
    public void driveToTarget(Pose2d target) {
        double dx = target.getX() - getPose().getX(); // get difference in x
        double dy = target.getY() - getPose().getY(); // get difference in y
        double l = Math.sqrt(dx*dx + dy*dy); // calculate distance
        dx /= l; dy /= l; // normalize to get the direction
        // calculate PID output and clamp
        double translationalOutput = Math.min(distanceController.calculate(l, 0.0), kTargetDriveMaxVelocity); 
        double vr = rotationController.calculate(NerdyMath.degreesToRadians(getAbsoluteHeadingDegrees()), target.getRotation().getRadians());
        vr = Math.min(vr, kTargetDriveMaxAngularVelocity);

        driveFieldOriented(dx * translationalOutput, dy * translationalOutput, vr);
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
        tab.add("Field Position", field).withSize(6,3);

        ///////////
        /// ALL ///
        ///////////
        for (Camera camera : Camera.values())
            Reportable.addCamera(tab, camera.name, camera.name, "http://" + camera.ip, LOG_LEVEL.ALL);

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
