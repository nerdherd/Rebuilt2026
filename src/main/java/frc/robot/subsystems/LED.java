package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

public class LED extends SubsystemBase {

    // CANdle hardware
    private final CANdle candle;

    // Colors
    private static final RGBWColor kGreen  = new RGBWColor(0, 217, 0, 0);
    private static final RGBWColor kWhite  = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
    private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
    private static final RGBWColor kRed    = RGBWColor.fromHex("#D9000000").orElseThrow();
    private static final RGBWColor kYellow = RGBWColor.fromHex("#D9000000").orElseThrow();

    // LED index ranges
    private static final int kSlot0StartIdx = 8;
    private static final int kSlot0EndIdx   = 37;
    private static final int kSlot1StartIdx = 38;
    private static final int kSlot1EndIdx   = 67;

    private static final double SHOOTER_READY_DURATION = 25.0;

    private double shooterReadyStartTime = -1.0;
    private boolean shooterReadyActive = false;

    public void startShooterReadyTimed() {
        shooterReadyStartTime = edu.wpi.first.wpilibj.DriverStation.getMatchTime();
        shooterReadyActive = true;
        setStatus(Status.SHOOTER_READY);
    }

    private enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    public enum Status {
        DISABLED,
        TELEOP,
        AUTO,
        DISCONNECTED,
        SHOOTER_READY,
        FEEDINGREADY,
    }

    private AnimationType m_anim0State = AnimationType.None;
    private AnimationType m_anim1State = AnimationType.None;
    private AnimationType currentAnimation = AnimationType.Rainbow;
    private AnimationType lastAnimation    = AnimationType.Rainbow;

    private final SendableChooser<AnimationType> m_anim0Chooser = new SendableChooser<>();
    private final SendableChooser<AnimationType> m_anim1Chooser = new SendableChooser<>();

    public LED(int candleID) {
        // Create CANdle on the RIO bus
        candle = new CANdle(candleID, CANBus.roboRIO());

        // Configure CANdle
        var cfg = new CANdleConfiguration();
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        candle.getConfigurator().apply(cfg);

        // Clear previous animations (all 8 slots)
        for (int i = 0; i < 8; ++i) {
            candle.setControl(new EmptyAnimation(i));
        }

        // Onboard LEDs solid colors
        candle.setControl(new SolidColor(0, 3).withColor(kGreen));
        candle.setControl(new SolidColor(4, 7).withColor(kWhite));

        // Chooser for slot 0 (strip 0)
        m_anim0Chooser.setDefaultOption("Color Flow", AnimationType.ColorFlow);
        m_anim0Chooser.addOption("Rainbow", AnimationType.Rainbow);
        m_anim0Chooser.addOption("Twinkle", AnimationType.Twinkle);
        m_anim0Chooser.addOption("Twinkle Off", AnimationType.TwinkleOff);
        m_anim0Chooser.addOption("Fire", AnimationType.Fire);

        // Chooser for slot 1 (strip 1)
        m_anim1Chooser.setDefaultOption("Larson", AnimationType.Larson);
        m_anim1Chooser.addOption("RGB Fade", AnimationType.RgbFade);
        m_anim1Chooser.addOption("Single Fade", AnimationType.SingleFade);
        m_anim1Chooser.addOption("Strobe", AnimationType.Strobe);
        m_anim1Chooser.addOption("Fire", AnimationType.Fire);

        SmartDashboard.putData("Animation 0", m_anim0Chooser);
        SmartDashboard.putData("Animation 1", m_anim1Chooser);
    }

    // Map robot state → status (call if you want it automatic)
    private void updateStatusFromRobot() {
        if (RobotState.isDisabled()) {
            setStatus(Status.DISABLED);
        } else if (RobotState.isTeleop()) {
            setStatus(Status.TELEOP);
        } else if (RobotState.isAutonomous()) {
            setStatus(Status.AUTO);
        }
    }

    // Public API for other code to set LED status
    public void setStatus(Status status) {
        switch (status) {
            case DISABLED:
            case TELEOP:
            case AUTO:
            case DISCONNECTED:
                changeAnimation(AnimationType.None);
                break;
            case SHOOTER_READY:
                changeAnimation(AnimationType.SingleFade);
                break;
            case FEEDINGREADY:
                changeAnimation(AnimationType.Strobe);
                break;
        }
    }

    // Actually apply the animation for strip 1 based on the enum
    public void changeAnimation(AnimationType newAnimation) {
        lastAnimation = currentAnimation;
        currentAnimation = newAnimation;
        m_anim1State = newAnimation;

        switch (m_anim1State) {
            default:
            case Larson:
                candle.setControl(
                    new LarsonAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                        .withColor(kRed)
                );
                break;
            case RgbFade:
                candle.setControl(
                    new RgbFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                );
                break;
            case SingleFade:
                candle.setControl(
                    new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                        .withColor(kGreen)
                );
                break;
            case Strobe:
                candle.setControl(
                    new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                        .withColor(kYellow)
                );
                break;
            case Fire:
                candle.setControl(
                    new FireAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                        .withDirection(AnimationDirectionValue.Backward)
                        .withCooling(0.4)
                        .withSparking(0.5)
                );
                break;
            case None:
                // Clear slot 1 animation
                candle.setControl(new EmptyAnimation(1));
                break;
        }
    }

    @Override
    public void periodic() {
        // Optional: keep LEDs matched to robot mode
        // updateStatusFromRobot();

        // Slot 0: driven by SmartDashboard chooser
        final var anim0Selection = m_anim0Chooser.getSelected();
        if (m_anim0State != anim0Selection) {
            m_anim0State = anim0Selection;

            switch (m_anim0State) {
                default:
                case ColorFlow:
                    candle.setControl(
                        new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                            .withColor(kViolet)
                    );
                    break;
                case Rainbow:
                    candle.setControl(
                        new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
                case Twinkle:
                    candle.setControl(
                        new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                            .withColor(kViolet)
                    );
                    break;
                case TwinkleOff:
                    candle.setControl(
                        new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                            .withColor(kViolet)
                    );
                    break;
                case Fire:
                    candle.setControl(
                        new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
                case None:
                    candle.setControl(new EmptyAnimation(0));
                    break;
            }
        // Existing chooser / animation code here...

    // Handle SHOOTER_READY timeout using match time
        if (shooterReadyActive) {
            double now = edu.wpi.first.wpilibj.DriverStation.getMatchTime();
            // matchTime counts DOWN, so compare difference
            if (shooterReadyStartTime - now >= SHOOTER_READY_DURATION) {
                shooterReadyActive = false;
                // Go back to whatever your normal status is (e.g. TELEOP)
                setStatus(Status.TELEOP);
        }
    }
}

