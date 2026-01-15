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
        if (!checkName(name)) return this;
        controllers.get(name).enableContinuousInput(minimumInput, maximumInput);
        return this;
    }
    
    public MultiProfiledPIDController withDisabledContinuousInput(String name) {
        if (!checkName(name)) return this;
        controllers.get(name).disableContinuousInput();
        return this;
    }
    
    public MultiProfiledPIDController withTolerance(String name, double errorTolerance, double derivativeTolerance) {
        if (!checkName(name)) return this;
        controllers.get(name).setTolerance(errorTolerance, derivativeTolerance);
        return this;
    }
    
    public ProfiledPIDController get(String name) {
        if (!checkName(name)) return null;
        return controllers.get(name);
    }
    
    public double calculate(String name, double measurement, double goal) {
        if (!checkName(name)) return 0.0;
        return controllers.get(name).calculate(measurement, goal);
    }
    
    public double calculate(String name, double measurement) {
        if (!checkName(name)) return 0.0;
        return controllers.get(name).calculate(measurement);
    }
    
    public boolean atGoal(String name) {
        if (!checkName(name)) return true;
        return controllers.get(name).atGoal();
    }

    public boolean atSetpoint(String name) {
        if (!checkName(name)) return true;
        return controllers.get(name).atSetpoint();
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

    private boolean checkName(String name) {
        if (!controllers.containsKey(name)) {
            DriverStation.reportError("MultiProfiledPIDController: " + name + " is not a valid entry!", true);
            return false;
        }
        return true;
    }
}
