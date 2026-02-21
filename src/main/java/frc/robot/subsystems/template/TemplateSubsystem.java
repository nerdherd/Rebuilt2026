// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.template;

import static frc.robot.Constants.LoggingConstants.kSubsystemTab;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable;

public class TemplateSubsystem extends SubsystemBase implements Reportable {
	/** primary motor; required */
	public final TalonFX motor1;
	/** secondary motor; optional */
	protected final TalonFX motor2;
	/** holds the configuration for both {@link #motor1} and {@link #motor2} */
	protected TalonFXConfiguration configuration;

	/** position controller for {@link SubsystemMode#POSITION} */
	private final MotionMagicVoltage positionController;
	/** velocity controller for {@link SubsystemMode#VELOCITY} */
	private final VelocityVoltage velocityController;
	/** voltage controller for {@link SubsystemMode#VOLTAGE} */
	private final VoltageOut voltageController;
	/** velocity controller for {@link SubsystemMode#PROFILED_VELOCITY} */
	private final MotionMagicVelocityVoltage profiledVelocityController;
	/** follower controller for {@link #motor2} */
	private final Follower followerController;

	/** brake/coast */
	private final NeutralOut neutralRequest = new NeutralOut();
	
	/** 
	 * target position or velocity depending on {@link SubsystemMode}
	 */
	private double desiredValue; 

	public final boolean useSubsystem;

	/** whether the subsystem will run {@link #periodic()} or use {@link #neutralRequest} */
	protected boolean enabled = false;

	/** used to indicate when the subsystem has an error, configured during debugging.  by default always false (reported at {@link LOG_LEVEL#ALL}) */
	public boolean _hasError = false;

	/** name of the subsystem */
	protected final String name;

	private final double defaultValue;

	public enum SubsystemMode {
		POSITION, 
		VELOCITY,
		VOLTAGE,
		PROFILED_VELOCITY
	}
	/** {@link SubsystemMode} of this subsystem */
	private final SubsystemMode mode;

	/**
	 * 
	 * @param name - used for {@link #shuffleboardTab}
	 * @param motor1ID - used for {@link #motor1}
	 * @param motor2ID - used for {@link #motor2}
	 * @param reverseMotor2 - whether to have {@link #motor2} oppose {@link #motor1}
	 * @param mode - the {@link SubsystemMode} for this subsystem
	 * @param defaultValue - initial position or velocity depending on {@link SubsystemMode}
	 */
	public TemplateSubsystem(String name, int motor1ID, int motor2ID, MotorAlignmentValue reverseMotor2, SubsystemMode mode, double defaultValue, boolean useSubsystem) {
		this.motor1 = getMotor(motor1ID);
		this.defaultValue = defaultValue;
		this.useSubsystem = useSubsystem;
		if (motor2ID != -1){
			this.motor2 = getMotor(motor2ID);
			followerController = new Follower(motor1ID, reverseMotor2);
		} else {
			this.motor2 = null;
			followerController = null;
		}
		this.mode = mode;
		this.desiredValue = defaultValue;

		switch (mode) {
			case POSITION:
				positionController 			= new MotionMagicVoltage(defaultValue);
				velocityController 			= null;
				voltageController  			= null;
				profiledVelocityController  = null;
				motor1.setPosition(defaultValue);
				break;
			case VELOCITY:
				positionController 			= null;
				velocityController 			= new VelocityVoltage(defaultValue);
				voltageController  			= null;
				profiledVelocityController  = null;
				break;
			case VOLTAGE:
				positionController 			= null;
				velocityController 			= null;
				voltageController  			= new VoltageOut(defaultValue);
				profiledVelocityController 	= null;
				break;
			case PROFILED_VELOCITY:
				positionController 			= null;
				velocityController 			= null;
				voltageController  			= null;
				profiledVelocityController 	= new MotionMagicVelocityVoltage(defaultValue);
				break;
			default:
				positionController 			= null;
				velocityController 			= null;
				voltageController  			= null;
				profiledVelocityController 	= null;
				break;
		}
		this.name = name;
		SuperSystem.registerSubsystem(this);
	}

	public TemplateSubsystem(String name, int motor1ID, SubsystemMode mode, double defaultValue, boolean useSubsystem){
		this(name, motor1ID, -1, MotorAlignmentValue.Aligned, mode, defaultValue, useSubsystem);
	}
	
	/** applies configuration to motors; should be used on construction */
	public TemplateSubsystem configureMotors(TalonFXConfiguration configuration){
		this.configuration = configuration;
		applyMotorConfigs();
		return this;
	}

	@Override
	public void periodic() {
		if(!enabled) {
			stop();
			return;
		}

		if(hasMotor2()) motor2.setControl(followerController);

		switch (mode) {
			case POSITION:
				motor1.setControl(positionController.withPosition(this.desiredValue));
				if (configuration.MotionMagic.MotionMagicCruiseVelocity == 0.0) NerdLog.reportWarning(name + ": MM Cruise Velocity is 0.0");
				if (configuration.MotionMagic.MotionMagicAcceleration == 0.0) NerdLog.reportWarning(name + ": MM Acceleration is 0.0");
				break;
			case VELOCITY:
				if (Math.abs(this.desiredValue) <= 0.1) {
					motor1.setControl(neutralRequest);
					if (hasMotor2()) motor2.setControl(neutralRequest);
				} else motor1.setControl(velocityController.withVelocity(this.desiredValue));
				break;
			case VOLTAGE:
				if (Math.abs(this.desiredValue) > 12)
					NerdLog.reportWarning(name + ": voltage > 12");
				motor1.setControl(voltageController.withOutput(this.desiredValue));
				break;
			case PROFILED_VELOCITY:
				motor1.setControl(profiledVelocityController.withVelocity(this.desiredValue).withAcceleration(configuration.MotionMagic.MotionMagicAcceleration));
				if (configuration.MotionMagic.MotionMagicAcceleration == 0.0) NerdLog.reportWarning(name + ": MM Acceleration is 0.0");
			default:
				break;
		}
	}

	// ------------------------------------ Helper Functions ------------------------------------ //
	
	/**
	 * gets a motor by trying to find it on the rio. if it isn't found, it tries again with the CANivore CANbus
	 * relies on completely unique ids for the entire robot network
	 * @param id - id of the motor to find
	 * @return the found motor
	 */
	private static TalonFX getMotor(int id) {
		TalonFX motor = new TalonFX(id);
		if (motor.getConnectedMotor().getValue().compareTo(ConnectedMotorValue.Unknown) == 0) 
			motor = new TalonFX(id, TunerConstants.kCANBus);
		return motor;
	}

	/**
	 * @return whether the subsystem is configured to have a second motor
	 */
	public boolean hasMotor2() {
		return motor2 != null;
	}

	/** used for logging, essentially returns the subsystem mode in string form */
	public String getFlavorText() {
		switch (mode) {
			case POSITION:
				return "Position";
			case PROFILED_VELOCITY:
			case VELOCITY:
				return "Velocity";
			case VOLTAGE:
				return "Voltage";
			default:
				break;
		}
		return "";
	}

	public String getUnit() {
		switch (mode) {
			case POSITION:
				return "rot";
			case PROFILED_VELOCITY:
			case VELOCITY:
				return "rps";
			case VOLTAGE:
				return "V";
			default:
				break;
		}
		return null;
	}

	/** applies motor configurations based on {@link #configuration} */
	public void applyMotorConfigs(){
		motor1.getConfigurator().apply(this.configuration);
		if(hasMotor2()) motor2.getConfigurator().apply(this.configuration);
	}

	/** sets and applies a {@link NeutralModeValue} to motors */
	public void setNeutralMode(NeutralModeValue mode){
		motor1.setNeutralMode(mode);
		if (hasMotor2()) motor2.setNeutralMode(mode);
	}
	
	/** brake or coast depending on current {@link NeutralModeValue} */
	public void stop(){
		this.enabled = false;
		motor1.setControl(neutralRequest);
		if (hasMotor2()) motor2.setControl(neutralRequest);
	}

	/** 
	 * enables or disables the subsystem 
	 * @see {@link #enabled} 
	 */
	public void setEnabled(boolean enabled){
		this.enabled = enabled;
		if (!this.enabled) stop();
	}

	/**
	 * disables the subsystem
	 */
	public void disable() {
		this.setEnabled(false);
	}
	
	/**
	 * enables the subsystem
	 */
	public void enable(){
		this.setEnabled(true);
	}

	/**
	 * sets {@link #desiredValue}
	 * @param value - value to set
	 */
	public void setDesiredValue(double value) {
		this.desiredValue = value;
	}

	/**
	 * @return {@link #desiredValue}
	 */
	public double getDesiredValue(){
		return desiredValue;
	}


	/**
	 * {@link #desiredValue}
	 * @return the position or velocity based on motor1
	 */
	public BaseStatusSignal getCurrentValue() {
		switch (mode) {
			case POSITION:
				return motor1.getPosition(false);
			case PROFILED_VELOCITY:
			case VELOCITY:
				return motor1.getVelocity(false);
			case VOLTAGE:
				return motor1.getMotorVoltage(false);
			default:
				break;
		}

		return null;
	}

	/**
	 * {@link #desiredValue}
	 * @return the position or velocity based on motor2
	 */
	public BaseStatusSignal getCurrentValue2() {
		if (!hasMotor2()) return null;
		switch (mode) {
			case POSITION:
				return motor2.getPosition(false);
			case PROFILED_VELOCITY:
			case VELOCITY:
				return motor2.getVelocity(false);
			case VOLTAGE:
				return motor2.getMotorVoltage(false);
			default:
				break;
		}

		return null;
	}

	/**
	 * could be useful for atPoint checks
	 * @return the current velocity of {@link #motor1} 
	 */
	public double getCurrentVelocity() {
		return motor1.getVelocity().getValueAsDouble();
	}

	/**
	 * @return temperature of {@link #motor1}
	 */
	public double getCurrentTemp(){
		return motor1.getDeviceTemp().getValueAsDouble();
	}

	/**
	 * @return temperature of {@link #motor2}
	 */
	public double getCurrentTemp2(){
		if (!hasMotor2()) return 0.0;
		return motor2.getDeviceTemp().getValueAsDouble();
	}

	public double getDefaultValue() {
		return defaultValue;
	}

	/**
	 * @return {@link #enabled}
	 */
	public boolean isEnabled() {
		return enabled;
	}

	// ------------------------------------ Command Functions ------------------------------------ //

	/**
	 * @param newEnabled
	 * @return {@link #enable()} in command form
	 */
	public Command setEnabledCommand(boolean newEnabled) {
		return Commands.runOnce(() -> setEnabled(newEnabled));
	}

	/**
	 * @param newValue
	 * @return {@link #setDesiredValue(double)} in command form
	 */
	public Command setDesiredValueCommand(double newValue) {
		return Commands.runOnce(() -> setDesiredValue(newValue));
	}

	/**
	 * @return {@link #stop()} in command form
	 */
	public Command stopCommand() {
		return Commands.runOnce(() -> stop());
	}

	// ------------------------------------ Logging Functions ------------------------------------ //

	/** 
	 * intialize shuffleboard logging on {@link #shuffleboardTab}
	 * @see {@link Reportable#addNumber(ShuffleboardTab, String, java.util.function.DoubleSupplier, frc.robot.util.logging.Reportable.LOG_LEVEL)}
	 * @see {@link Reportable#addBoolean(ShuffleboardTab, String, java.util.function.BooleanSupplier, frc.robot.util.logging.Reportable.LOG_LEVEL)}
	 * @see {@link Reportable#addString(ShuffleboardTab, String, java.util.function.Supplier, frc.robot.util.logging.Reportable.LOG_LEVEL)}
	 */
    public void initializeLogging(){
        ///////////
        /// ALL ///
        ///////////
		NerdLog.logData(kSubsystemTab + name, "Commands", this, LOG_LEVEL.ALL); 
		
        NerdLog.logNumber(kSubsystemTab + name,"Desired " + getFlavorText(), () -> getDesiredValue(), getUnit(), Reportable.LOG_LEVEL.ALL);
		NerdLog.logBoolean(kSubsystemTab + name, "Has Error", () -> _hasError, Reportable.LOG_LEVEL.ALL);

		NerdLog.logNumber(kSubsystemTab + name, "Torque Current 1", motor1.getTorqueCurrent(false), motor1.getNetwork().getName(), Reportable.LOG_LEVEL.ALL);
		if (hasMotor2()) NerdLog.logNumber(kSubsystemTab + name, "Torque Current 2", motor2.getTorqueCurrent(false), motor1.getNetwork().getName(), Reportable.LOG_LEVEL.ALL);

		NerdLog.logNumber(kSubsystemTab + name, "Supply Current 1", motor1.getSupplyCurrent(false), motor1.getNetwork().getName(), Reportable.LOG_LEVEL.ALL);
		if (hasMotor2()) NerdLog.logNumber(kSubsystemTab + name, "Supply Current 2", motor2.getSupplyCurrent(false), motor1.getNetwork().getName(), Reportable.LOG_LEVEL.ALL);

		//////////////
		/// MEDIUM ///
        //////////////
        NerdLog.logBoolean(kSubsystemTab + name, "Enabled", () -> this.enabled, Reportable.LOG_LEVEL.MEDIUM);
        NerdLog.logNumber(kSubsystemTab + name, "Temperature 1", motor1.getDeviceTemp(false), motor1.getNetwork().getName(), Reportable.LOG_LEVEL.MEDIUM);
        if (hasMotor2()) NerdLog.logNumber(kSubsystemTab + name, "Temperature 2", motor2.getDeviceTemp(false), motor1.getNetwork().getName(), Reportable.LOG_LEVEL.MEDIUM);
        
        //////////////
        /// MINIMAL //
        //////////////
        NerdLog.logNumber(kSubsystemTab + name, getFlavorText() + " 1", getCurrentValue(), motor1.getNetwork().getName(), Reportable.LOG_LEVEL.MINIMAL);
        if (hasMotor2()) NerdLog.logNumber(kSubsystemTab + name, getFlavorText() + " 2", getCurrentValue2(), motor1.getNetwork().getName(), Reportable.LOG_LEVEL.MINIMAL);
    }
}
