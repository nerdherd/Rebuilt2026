// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.HashMap;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * i didn't like the current implementation of the PID controller
 */
public class MultiProfiledPIDController {
    private final HashMap<String, ProfiledPIDController> controllers = new HashMap<>();

    public MultiProfiledPIDController add(String name, PIDConstants pidConstants, Constraints profileConstraints, double errorTolerance, double derivativeTolerance) {
        ProfiledPIDController controller = new ProfiledPIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD, profileConstraints);
        controller.setTolerance(errorTolerance, derivativeTolerance);
        controllers.put(name, controller);
        return this;
    }
    
    public MultiProfiledPIDController withContinuousInput(String name, double minimumInput, double maximumInput) {
        controllers.get(name).enableContinuousInput(minimumInput, maximumInput);
        return this;
    }

    public MultiProfiledPIDController withDisabledContinuousInput(String name) {
        controllers.get(name).disableContinuousInput();
        return this;
    }
    
    public MultiProfiledPIDController withTolerance(String name, double errorTolerance, double derivativeTolerance) {
        controllers.get(name).setTolerance(errorTolerance, derivativeTolerance);
        return this;
    }

    public ProfiledPIDController get(String name) {
        return controllers.get(name);
    }

    public double calculate(String name, double measurement, double goal) {
        return controllers.get(name).calculate(measurement, goal);
    }

    public double calculate(String name, double measurement) {
        return controllers.get(name).calculate(measurement);
    }

    public void reset(double... measurements) {
        if (measurements.length < controllers.size()) {
            DriverStation.reportError("MultiProfiledPIDController: not enough reset measurements!", true);
            return;
        }
        int i = 0;       
        for (String controller : controllers.keySet()) {
            controllers.get(controller).reset(measurements[i]);
            i++;
        }
    }
    
    public int size() {
        return controllers.size();
    }
}
