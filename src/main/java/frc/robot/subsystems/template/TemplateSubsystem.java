// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.template;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Reportable;

public abstract class TemplateSubsystem extends SubsystemBase implements Reportable {
	/** primary motor; required */
	private final TalonFX motor1;
	/** secondary motor; optional */
	private final TalonFX motor2;
	/** holds the configuration for both {@link #motor1} and {@link #motor2} */
	private TalonFXConfiguration configuration;

	/** position controller for {@link SubsystemMode#POSITION} */
	private final MotionMagicVoltage positionController;
	/** velocity controller for {@link SubsystemMode#VELOCITY} */
	private final VelocityVoltage velocityController;
	/** follower controller for {@link #motor2} */
	private final Follower followerController;

	/** brake/coast */
	private final NeutralOut neutralRequest = new NeutralOut();

	
	/** 
	 * target position or velocity depending on {@link SubsystemMode}
	 */
	private double desiredValue; 

	/** whether the subsystem will run {@link #periodic()} or use {@link #neutralRequest} */
	private boolean enabled = false;

	/** shuffleboard tab for logging, named through the constructor */
	private final ShuffleboardTab shuffleboardTab;
	
	public enum SubsystemMode {
		POSITION, 
		VELOCITY
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
	public TemplateSubsystem(String name, int motor1ID, int motor2ID, boolean reverseMotor2, SubsystemMode mode, double defaultValue) {
		this.motor1 = new TalonFX(motor1ID);
		if (motor2ID != -1){
			this.motor2 = new TalonFX(motor2ID);
			followerController = new Follower(motor1ID, reverseMotor2);
		}else{
			this.motor2 = null;
			followerController = null;
		}
		this.mode = mode;
		this.desiredValue = defaultValue;
		if (this.mode == SubsystemMode.POSITION){
			positionController = new MotionMagicVoltage(defaultValue);
			velocityController = null;
		}else{
			positionController = null;
			velocityController = new VelocityVoltage(defaultValue);
		}
		shuffleboardTab = Shuffleboard.getTab(name);
	}

	public TemplateSubsystem(String name, int motor1ID, SubsystemMode mode, double defaultValue){
		this(name, motor1ID, -1, false, mode, defaultValue);
	}
	
	/** applies configuration to motors; should be used on construction */
	public void configureMotors(TalonFXConfiguration configuration){
		this.configuration = configuration;
		applyMotorConfigs();
	}

	@Override
	public void periodic() {
		if(!enabled){
			stop();
			return;
		}

		if(mode == SubsystemMode.POSITION) 
			motor1.setControl(positionController.withPosition(this.desiredValue));
		else if(mode == SubsystemMode.VELOCITY) 
			motor1.setControl(velocityController.withVelocity(this.desiredValue));

		if(motor2 != null) motor2.setControl(followerController);
	}

	// ------------------------------------ Helper Functions ------------------------------------ //
	
	/** applies motor configurations based on {@link #configuration} */
	private void applyMotorConfigs(){
		motor1.getConfigurator().apply(this.configuration);
		if(motor2 != null) motor2.getConfigurator().apply(this.configuration);
	}

	/** sets and applies a {@link NeutralModeValue} to motors */
	public void setNeutralMode(NeutralModeValue mode){
		this.configuration.MotorOutput.NeutralMode = mode;
		applyMotorConfigs();
	}
	
	/** brake or coast depending on current {@link NeutralModeValue} */
	public void stop(){
		motor1.setControl(neutralRequest);
		if (motor2 != null) motor2.setControl(neutralRequest);
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
	public double getCurrentValue() {
		return (mode == SubsystemMode.POSITION) ? motor1.getPosition().getValueAsDouble() : motor1.getVelocity().getValueAsDouble();
	}

	/**
	 * {@link #desiredValue}
	 * @return the position or velocity based on motor2
	 */
	public double getCurrentValueMotor2() {
		if (motor2 == null) return 0.0;
		return (mode == SubsystemMode.POSITION) ? motor2.getPosition().getValueAsDouble() : motor2.getVelocity().getValueAsDouble();
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
	 * @see {@link Reportable#addNumber(ShuffleboardTab, String, java.util.function.DoubleSupplier, frc.robot.subsystems.Reportable.LOG_LEVEL)}
	 * @see {@link Reportable#addBoolean(ShuffleboardTab, String, java.util.function.BooleanSupplier, frc.robot.subsystems.Reportable.LOG_LEVEL)}
	 * @see {@link Reportable#addString(ShuffleboardTab, String, java.util.function.Supplier, frc.robot.subsystems.Reportable.LOG_LEVEL)}
	 */
	public abstract void initializeLogging();
}
