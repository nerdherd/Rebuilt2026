package frc.robot.subsystems;

import static frc.robot.Constants.LoggingConstants.kSubsystemTab;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants.LEDSegments;
import frc.robot.generated.TunerConstants;
import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable;

public class LED extends SubsystemBase implements Reportable {
    private final CANdle candle;
    private final CANdleConfiguration configuration;
    private final ArrayList<EmptyAnimation> resetAnimations = new ArrayList<>();

    /**
     * @param id - CAN ID of CANdle
     * @param configuration
     */
    public LED(int id, CANdleConfiguration configuration) {
		CANdle candle = new CANdle(id);
		if (!candle.isConnected()) {
			candle.close();
			candle = new CANdle(id, TunerConstants.kCANBus);
		}
        this.candle = candle;
        this.configuration = configuration;
        for (int i = 0; i < 8; i++) resetAnimations.add(new EmptyAnimation(i));
    }

    public void reapplyConfigs() {
        this.candle.getConfigurator().apply(configuration);
    }

    public StatusCode setControl(ControlRequest request) {
        return this.candle.setControl(request);
    }

    /**
     * clears the entire strip to black, and fills slots with empty
     */
    public void clear() {
        reset();
        setControl(new SolidColor(LEDSegments.ALL.start, LEDSegments.ALL.end));
    }
    
    /** 
     * resets all slots
     */
    public void reset() {
        // for (int i = 1; i < 8; i++) disable(i);
        candle.clearAllAnimations();
    }

    /**
     * disables particular slot
     * @param index - index of slot
     */
    public void disable(int index) {
        if (index < 0 || index > 7) return;
        setControl(resetAnimations.get(index));
    }

    @Override
    public void initializeLogging() {
        NerdLog.logSignal(kSubsystemTab + "CANdle/Temperature", candle.getDeviceTemp(false), candle.getNetwork().getName(), LOG_LEVEL.MINIMAL);
        NerdLog.logBoolean(kSubsystemTab + "CANdle/Connected", candle::isConnected, LOG_LEVEL.MINIMAL);
    }
}