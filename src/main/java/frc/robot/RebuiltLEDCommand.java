// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.LEDConstants.Animations;

import frc.robot.Constants.LEDConstants.Colors;
import frc.robot.Constants.LEDConstants.LEDSegments;
import frc.robot.subsystems.LED;

public class RebuiltLEDCommand extends Command {
  public final LED led;
  private int throttleTracker = 0;
  private Supplier<Double> countdownSupplier = () -> 0.0, shooterSupplier = () -> 0.0;
  private Supplier<Boolean> intakeSupplier = () -> false;

  public RebuiltLEDCommand(LED led) {
    this.led = led; 
    addRequirements(led);
  }

  public void registerCountdownSupplier(Supplier<Double> s) {countdownSupplier = s;}
  public void registerShooterSupplier(Supplier<Double> s) {shooterSupplier = s;}
  public void registerIntakeSupplier(Supplier<Boolean> s) {intakeSupplier = s;}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    ++throttleTracker;
    if (throttleTracker < 10) return;
    throttleTracker = 0;
    led.reset();
    if (!DriverStation.isDSAttached()) { // when not conected
      led.setControl(Animations.kDisconnectedVertical);
      led.setControl(Animations.kDisconnectedHorizontal);
    } else if (DriverStation.isDisabled()) { // when connected and not enabled
      led.setControl(Animations.kDisconnectedVertical);
      led.setControl(Animations.kDisabled);
    } else {
      // horizontal during match
      double cVal = countdownSupplier.get();
      int cMid = (int)((LEDSegments.HORIZONTAL.end - LEDSegments.VERTICAL.start) * cVal) + LEDSegments.HORIZONTAL.start;
      led.setControl(Animations.kDefaultHorizontal
        .withColor((intakeSupplier.get() ? Colors.kCYAN : Colors.kNERDHERD_BLUE)
            .scaleBrightness(getPulseBrightness()))
        .withLEDStartIndex(cMid)
        .withLEDEndIndex(LEDSegments.HORIZONTAL.end)
        );
      if (cMid != LEDSegments.VERTICAL.start)
        led.setControl(Animations.kCountdown
          .withLEDStartIndex(LEDSegments.HORIZONTAL.start)
          .withLEDEndIndex(cMid - 1)
          );
      
      // vertical during match
      if (DriverStation.isAutonomousEnabled()) { // autonomous fire
        led.setControl(Animations.kAutonomousVertical);
      } else {
        double val = shooterSupplier.get();
        if (val > 0.999) // full bar
          led.setControl(Animations.kTeleopVertical
            .withColor(Colors.kGREEN.scaleBrightness(getPulseBrightness()))
            .withLEDStartIndex(LEDSegments.VERTICAL.start)
            .withLEDEndIndex(LEDSegments.VERTICAL.end)
            );
        else { // not so full bar
          int mid = (int)((LEDSegments.VERTICAL.end - LEDSegments.VERTICAL.start) * val) + LEDSegments.VERTICAL.start;
          led.setControl(Animations.kTeleopVertical
            .withColor(Colors.kNERDHERD_BLUE.scaleBrightness(getPulseBrightness()))
            .withLEDStartIndex(mid)
            .withLEDEndIndex(LEDSegments.VERTICAL.end)
            );
          if (mid != LEDSegments.VERTICAL.start) 
            led.setControl(Animations.kShooterRamp
              .withLEDStartIndex(LEDSegments.VERTICAL.start)
              .withLEDEndIndex(mid - 1)
            );
        }
      }
    }
  }

  public double getPulseBrightness() {
    return Math.abs(Math.sin(MathSharedStore.getTimestamp())) * 0.2 + 0.8;
  }

  @Override
  public void end(boolean interrupted) {
    led.clear();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
