// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public interface Reportable {
	public enum LOG_LEVEL {
		ALL(4),
		MEDIUM(3),
		MINIMAL(2),
		NONE(1)
		;

		public int level = 0;
		LOG_LEVEL(int level) { this.level = level; }
	}

	public void initializeLogging();

	static public void addNumber(ShuffleboardTab shuffleboardTab, String name, DoubleSupplier supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.compareTo(loggingLevel) <= 0) shuffleboardTab.addNumber(name, supplier);
	}

	static public void addBoolean(ShuffleboardTab shuffleboardTab, String name, BooleanSupplier supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.compareTo(loggingLevel) <= 0) shuffleboardTab.addBoolean(name, supplier);
	}

	static public void addString(ShuffleboardTab shuffleboardTab, String name, Supplier<String> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.compareTo(loggingLevel) <= 0) shuffleboardTab.addString(name, supplier);
	}

	static public void add(ShuffleboardTab shuffleboardTab, String name, Sendable sendable, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.compareTo(loggingLevel) <= 0) shuffleboardTab.add(name, sendable);
	}

	/**
	 * idk either ¯\_(ツ)_/¯
	 * @param shuffleboardTab
	 * @param cameraReadableName
	 * @param cameraInternalName
	 * @param IP
	 * @param loggingLevel
	 */
	static public void addCamera(ShuffleboardTab shuffleboardTab, String cameraReadableName, String cameraInternalName, String IP, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.level >= loggingLevel.level) shuffleboardTab.addCamera(cameraReadableName, cameraInternalName, IP);
	}
}
