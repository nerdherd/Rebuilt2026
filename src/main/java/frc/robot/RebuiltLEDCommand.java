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
    if (DriverStation.isDSAttached()) {
      led.setControl(Animations.kDisconnectedVertical);
      led.setControl(Animations.kDisconnectedHorizontal);
    } else if (DriverStation.isDisabled()) {
      led.setControl(Animations.kDisconnectedVertical);
      led.setControl(Animations.kDisabled);
    } else {
      led.setControl(Animations.kDefaultHorizontal
        .withColor((intakeSupplier.get() ? Colors.kCYAN : Colors.kNERDHERD_BLUE)
            .scaleBrightness(getPulseBrightness())));
      if (DriverStation.isAutonomousEnabled()) {
        led.setControl(Animations.kAutonomousVertical);
      } else {
        double val = shooterSupplier.get();
        if (val > 0.999) 
          led.setControl(Animations.kTeleopVertical
            .withColor(Colors.kGREEN.scaleBrightness(getPulseBrightness()))
            .withLEDStartIndex(LEDSegments.VERTICAL.startIndex)
            .withLEDEndIndex(LEDSegments.VERTICAL.endIndex)
            );
        else {
          led.setControl(Animations.kTeleopVertical
            .withColor(Colors.kNERDHERD_BLUE.scaleBrightness(getPulseBrightness()))
            .withLEDStartIndex()
            .withLEDEndIndex(LEDSegments.VERTICAL.endIndex)
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
