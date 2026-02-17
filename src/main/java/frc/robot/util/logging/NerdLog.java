package frc.robot.util.logging;

import static frc.robot.Constants.ROBOT_LOG_LEVEL;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.util.sendable.Sendable;
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

	public static void logBoolean(String key, String name, Supplier<Boolean> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());
		Runnable logger = () -> {
			DogLog.log(key + "/" + name, supplier.get());
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

	public static void logData(String key, String name, Supplier<Sendable> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());
		Runnable logger = () -> {
			SmartDashboard.putData(key + "/" + name, supplier.get());
		};
		logSuppliers.get(loggingLevel).add(logger);
	}
}
