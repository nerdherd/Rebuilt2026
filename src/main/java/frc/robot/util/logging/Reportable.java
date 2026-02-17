// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.logging;

public interface Reportable {
	public enum LOG_LEVEL {
		ALL(100),
		MEDIUM(10),
		MINIMAL(1),
		NONE(1)
		;

		public final int throttle;
		LOG_LEVEL(int throttle) { this.throttle = throttle; }
	}

	public void initializeLogging();
}
