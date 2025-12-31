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
import static frc.robot.Constants.SwerveDriveConstants.kTowSwerveRequest;

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
import edu.wpi.first.wpilibj2.command.Subsystem;
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

        setVision(USE_VISION);
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
     * @param activate whether to activate or deactivate
     */
    private void setVision(boolean activate) {
        for (Camera camera : Camera.values())
            LimelightHelpers.setPipelineIndex(camera.name, (activate) ? 1 : 0);
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
