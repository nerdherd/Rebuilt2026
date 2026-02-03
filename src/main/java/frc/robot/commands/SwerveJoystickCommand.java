package frc.robot.commands;

import static frc.robot.Constants.SwerveDriveConstants.kDriveAlpha;
import static frc.robot.Constants.SwerveDriveConstants.kMinimumMotorOutput;
import static frc.robot.Constants.SwerveDriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
import static frc.robot.Constants.SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
import static frc.robot.Constants.SwerveDriveConstants.kTeleMaxAcceleration;
import static frc.robot.Constants.SwerveDriveConstants.kTeleMaxDeceleration;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.util.NerdyMath;
public class SwerveJoystickCommand extends Command {
    private final NerdDrivetrain swerveDrive;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> useFieldOriented;
    private final Supplier<Boolean> useTowMode;
    private final Supplier<Boolean> usePrecisionMode;
    private final Supplier<Double> desiredAngle;
    private final Supplier<Boolean> turnToAngleSupplier;
    private final PIDController turnToAngleController;
    private final Supplier<Translation2d> dPadDirectionalSupplier;
    // private Filter xFilter, yFilter, turningFilter;

    public static double targetAngle = 0;

    /**
     * Construct a new SwerveJoystickCommand
     * 
     * @param swerveDrive           The Swerve Drive subsystem
     * @param xSpdFunction          A supplier returning the desired x speed
     * @param ySpdFunction          A supplier returning the desired y speed
     * @param turningSpdFunction    A supplier returning the desired turning speed
     * @param fieldOrientedFunction A boolean supplier that toggles field oriented/robot oriented mode.
     * @param towSupplier           A boolean supplier that toggles the tow mode.
     * @param precisionSupplier     A boolean supplier that toggles the precision mode.
     */
    public SwerveJoystickCommand(NerdDrivetrain swerveDrive,
            Supplier<Double> xSpdFunction, 
            Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, 
            Supplier<Boolean> towSupplier, 
            Supplier<Boolean> precisionSupplier,
            Supplier<Boolean> turnToAngleSupplier,
            Supplier<Double> desiredAngleSupplier,
            Supplier<Translation2d> dPadDirectionalSupplier
        ) {
        this.swerveDrive = swerveDrive;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.useFieldOriented = fieldOrientedFunction;
        this.useTowMode = towSupplier;
        this.usePrecisionMode = precisionSupplier;

        
        this.turnToAngleSupplier = turnToAngleSupplier;
        this.desiredAngle = desiredAngleSupplier;


        this.dPadDirectionalSupplier = dPadDirectionalSupplier;


        // this.xFilter = new OldDriverFilter2(
        //     ControllerConstants.kDeadband, 
        //     kMinimumMotorOutput,
        //     kTeleDriveMaxSpeedMetersPerSecond, 
        //     kDriveAlpha, 
        //     kTeleMaxAcceleration, 
        //     kTeleMaxDeceleration);
        // this.yFilter = new OldDriverFilter2(
        //     ControllerConstants.kDeadband, 
        //     kMinimumMotorOutput,
        //     kTeleDriveMaxSpeedMetersPerSecond, 
        //     kDriveAlpha, 
        //     kTeleMaxAcceleration, 
        //     kTeleMaxDeceleration);
        // this.turningFilter = new FilterSeries(
        //     new DeadbandFilter(ControllerConstants.kRotationDeadband),
        //     new ScaleFilter(kTeleDriveMaxAngularSpeedRadiansPerSecond)
        //     );
        

        this.turnToAngleController = new PIDController(
            SwerveDriveConstants.kPThetaAuto,
            SwerveDriveConstants.kIThetaAuto,
            SwerveDriveConstants.kDThetaAuto
            );

        // this.turnToAngleController = new PIDController(
        //     SwerveAutoConstants.kPTurnToAngle, 
        //     SwerveAutoConstants.kITurnToAngle, 
        //     SwerveAutoConstants.kDTurnToAngle, 
        //     0.02);
        
        this.turnToAngleController.setTolerance(
            SwerveDriveConstants.kTurnToAnglePositionToleranceAngle, 
            SwerveDriveConstants.kTurnToAngleVelocityToleranceAnglesPerSec * 0.02);
    

        this.turnToAngleController.enableContinuousInput(0, 360);

        addRequirements(swerveDrive);
    }

    private double lastExponentialSmoothX = 0.;
    private double lastExponentialSmoothY = 0.;
    public double translationFilter(double x, boolean isX){
        // deadband filter
        if (Math.abs(x)<Math.abs(ControllerConstants.kDeadband)) return 0.;
        // scale filter
        x*=1.7;
        // clamp filter
        x = NerdyMath.clamp(x,-1.,1.);
        // wrapper filter
        x -= ControllerConstants.kDeadband*Math.signum(x);
        // exponential smoothing kDriveAlpha
        if (isX) {
            x = (kDriveAlpha*x) + ((1-kDriveAlpha)*lastExponentialSmoothX);
            lastExponentialSmoothX = x;
        } else {
            x = (kDriveAlpha*x) + ((1-kDriveAlpha)*lastExponentialSmoothY);
            lastExponentialSmoothY = x;
        }
        // power filter
        x = Math.signum(x) * Math.abs(Math.pow(x,3));
        // wrapper filter
        x = Math.signum(x) * ((Math.abs(x) / SwerveDriveConstants.kDeadbandScaler) + ControllerConstants.kDeadband);
        // wrapper filter
        if (isX) x = Math.signum(x) * SwerveDriveConstants.kslewRateLimiterX.calculate(Math.abs(x));
        else x = Math.signum(x) * SwerveDriveConstants.kslewRateLimiterY.calculate(Math.abs(x));
        // scale filter
        x*= kTeleDriveMaxSpeedMetersPerSecond;
        // clamp filter
        x = NerdyMath.clamp(x,-kTeleDriveMaxSpeedMetersPerSecond,kTeleDriveMaxSpeedMetersPerSecond);
        return x;
    }

    public static double turningFilter(double x){
        if (Math.abs(x)<Math.abs(ControllerConstants.kRotationDeadband)) return 0;
        return x*kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (useTowMode.get()) {
            swerveDrive.setControl(SwerveDriveConstants.kTowSwerveRequest);
            return;
        }

        // get speeds
        double turningSpeed;
        double xSpeed = xSpdFunction.get();
        double ySpeed = -ySpdFunction.get();

        /** rad/s */
        double filteredTurningSpeed;
        /** m/s */
        double filteredXSpeed = translationFilter(xSpeed,true); // TODO reconsider filters
        /** m/s */
        double filteredYSpeed = translationFilter(ySpeed,false);

        if (turnToAngleSupplier.get()) {
            double tempAngle = desiredAngle.get();
            if ((Math.abs(tempAngle - 1000.0) > 0.01)) {
                targetAngle = tempAngle;
            } else {
                targetAngle = ((targetAngle + turningSpdFunction.get() % 360) + 360) % 360;
                // SwerveDriveConstants.kPThetaTeleop.loadPreferences();
                // SwerveDriveConstants.kIThetaTeleop.loadPreferences();
                // SwerveDriveConstants.kDThetaTeleop.loadPreferences();
                // turnToAngleController.setP(SwerveDriveConstants.kPThetaTeleop.get());
                // turnToAngleController.setI(SwerveDriveConstants.kIThetaTeleop.get());
                // turnToAngleController.setD(SwerveDriveConstants.kDThetaTeleop.get());
            }
            // todo, since we use field ori control, better to turn to field 0,90, 180,270
            turningSpeed = turnToAngleController.calculate(swerveDrive.getOperatorHeadingDegrees(), targetAngle);
            SmartDashboard.putNumber("Turning Speed Initial", turningSpeed);
            // turningSpeed += Math.signum(turningSpeed) * SwerveAutoConstants.kTurnToAngleFeedForwardDegreesPerSecond;
            turningSpeed = Math.toRadians(turningSpeed);
            turningSpeed = MathUtil.clamp(
                turningSpeed, 
                -SwerveDriveConstants.kTurnToAngleMaxAngularSpeedRadiansPerSecond, 
                SwerveDriveConstants.kTurnToAngleMaxAngularSpeedRadiansPerSecond);
            SmartDashboard.putNumber("Turning Speed", turningSpeed);
            SmartDashboard.putNumber("Target Angle", targetAngle);
            
            filteredTurningSpeed = turningSpeed;
        }
        else {
            // Manual turning
            turningSpeed = turningSpdFunction.get();
            turningSpeed *= -0.5; // TODO WHY?
            filteredTurningSpeed = turningFilter(turningSpeed);
        }


        if (usePrecisionMode.get()) {
            filteredXSpeed /= 4;
            filteredYSpeed /= 4;
            filteredTurningSpeed /= 2; // Also slows down the turn to angle speed
        }
               
        Translation2d dPad = dPadDirectionalSupplier.get();
        if (!dPad.equals(Translation2d.kZero)) {
            swerveDrive.driveRobotOriented(dPad.getX() * 1.0, dPad.getY() * 1.0, 0.0);
        } else if (useFieldOriented.get()) {
            swerveDrive.driveFieldOriented(filteredXSpeed, filteredYSpeed, filteredTurningSpeed);
        } else {
            swerveDrive.driveRobotOriented(filteredXSpeed, filteredYSpeed, filteredTurningSpeed);
        }
    } 

    public double getTargetAngle() {
        return targetAngle;
    }
}