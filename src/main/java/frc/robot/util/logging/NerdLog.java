package frc.robot.util.logging;

import static frc.robot.Constants.ROBOT_LOG_LEVEL;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.util.logging.Reportable.LOG_LEVEL;

public class NerdLog {
    private static HashMap<Reportable.LOG_LEVEL, ArrayList<Runnable>> logSuppliers = new HashMap<>();
	private static double timeLastPublished = 0.0;
	private static int publishCount = 0;

	public static void periodic() {
		double currentTime = MathSharedStore.getTimestamp();
		if (currentTime - timeLastPublished >= LoggingConstants.LOGGING_INTERVAL) {
			LOG_LEVEL[] levels = LOG_LEVEL.values();
			for (int i = ROBOT_LOG_LEVEL.ordinal(); i < levels.length; i++) {
				if (publishCount % levels[i].throttle == 0 && logSuppliers.containsKey(levels[i])) 
					for (Runnable log : logSuppliers.get(levels[i]))
						log.run();
			}
			publishCount++;
			timeLastPublished = currentTime;
		}
	}

    public static void logNumber(String key, String name, Supplier<Double> supplier, String unit, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());
		Runnable logger = () -> {
			DogLog.log(key + "/" + name, supplier.get(), unit);
		};
		logSuppliers.get(loggingLevel).add(logger);
	}

	public static void logNumber(String key, String name, Supplier<Double> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());
		Runnable logger = () -> {
			DogLog.log(key + "/" + name, supplier.get());
		};
		logSuppliers.get(loggingLevel).add(logger);
	}

	public static void logNumberArray(String key, String name, Supplier<Double[]> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());
		Runnable logger = () -> {
			DogLog.log(key + "/" + name, Arrays.stream(supplier.get()).mapToDouble(Double::doubleValue).toArray());
		};
		logSuppliers.get(loggingLevel).add(logger);
	}

	public static void logNumberArray(String key, String name, Supplier<Double[]> supplier, String unit, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());
		Runnable logger = () -> {
			DogLog.log(key + "/" + name, Arrays.stream(supplier.get()).mapToDouble(Double::doubleValue).toArray(), unit);
		};
		logSuppliers.get(loggingLevel).add(logger);
	}

	public static void logBoolean(String key, String name, Supplier<Boolean> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());
		Runnable logger = () -> {
			DogLog.log(key + "/" + name, supplier.get());
		};
		logSuppliers.get(loggingLevel).add(logger);
	}

	public static void logBooleanArray(String key, String name, Supplier<Boolean[]> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());
		Runnable logger = () -> {
			Boolean[] objectArray = supplier.get();
			boolean[] array = new boolean[objectArray.length];
			for (int i = 0; i < objectArray.length; i++) array[i] = objectArray[i].booleanValue();
			DogLog.log(key + "/" + name, array);
		};
		logSuppliers.get(loggingLevel).add(logger);
	}

	public static void logString(String key, String name, Supplier<String> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());
		Runnable logger = () -> {
			DogLog.log(key + "/" + name, supplier.get());
		};
		logSuppliers.get(loggingLevel).add(logger);
	}
	
	public static void logStringArray(String key, String name, Supplier<String[]> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());
		Runnable logger = () -> {
			DogLog.log(key + "/" + name, supplier.get());
		};
		logSuppliers.get(loggingLevel).add(logger);
	}

	public static void logData(String key, String name, Sendable supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		SmartDashboard.putData(key + "/" + name, supplier);
	}

	public static void logStructSerializable(String key, String name, Supplier<StructSerializable> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());
		Runnable logger = () -> {
			DogLog.forceNt.log(key + "/" + name, supplier.get());
		};
		logSuppliers.get(loggingLevel).add(logger);
	}

	public static void logSwerveModules(String key, String name, Supplier<SwerveDriveState> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		SmartDashboard.putData(key + "/" + name, generateModuleSendable(supplier));
	}

	private static Sendable generateModuleSendable(Supplier<SwerveDriveState> state) {
		return new Sendable() {
			@Override
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("SwerveDrive");

				builder.addDoubleProperty("Front Left Angle", () -> state.get().ModuleStates[0].angle.getRadians(), null);
				builder.addDoubleProperty("Front Left Velocity", () -> state.get().ModuleStates[0].speedMetersPerSecond, null);

				builder.addDoubleProperty("Front Right Angle", () -> state.get().ModuleStates[1].angle.getRadians(), null);
				builder.addDoubleProperty("Front Right Velocity", () -> state.get().ModuleStates[1].speedMetersPerSecond, null);

				builder.addDoubleProperty("Back Left Angle", () -> state.get().ModuleStates[2].angle.getRadians(), null);
				builder.addDoubleProperty("Back Left Velocity", () -> state.get().ModuleStates[2].speedMetersPerSecond, null);

				builder.addDoubleProperty("Back Right Angle", () -> state.get().ModuleStates[3].angle.getRadians(), null);
				builder.addDoubleProperty("Back Right Velocity", () -> state.get().ModuleStates[3].speedMetersPerSecond, null);

				builder.addDoubleProperty("Robot Angle", () -> state.get().Pose.getRotation().getRadians(), null);
			}
		};
	}

	public static void print(String message) {
		DogLog.logFault(message, AlertType.kInfo);
		DriverStation.reportWarning(message, false);
	}
	
	public static void reportWarning(String message) {
		DogLog.logFault(message, AlertType.kWarning);
		DriverStation.reportWarning(message, true);
	}
	
	public static void reportError(String message) {
		DogLog.logFault(message, AlertType.kError);
		DriverStation.reportWarning(message, true);
	}
}
