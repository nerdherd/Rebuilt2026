package frc.robot.subsystems;

import static frc.robot.Constants.Subsystems.*;

import java.util.ArrayList;
import java.util.function.Consumer;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.template.TemplateSubsystem;

public class SuperSystem implements Reportable {
    public static final ArrayList<TemplateSubsystem> subsystems = new ArrayList<>();
    public NerdDrivetrain swerveDrivetrain;

    public SuperSystem(NerdDrivetrain swerveDrivetrain) {
        this.swerveDrivetrain = swerveDrivetrain;
    }
    
    public static void registerSubsystem(TemplateSubsystem subsystem) {
        subsystems.add(subsystem);
    }

    public void applySubsystems(Consumer<TemplateSubsystem> f) {
        for (TemplateSubsystem subsystem : subsystems) f.accept(subsystem);
    }

    // ------------------------------------ subsystems ------------------------------------ //
    public void reConfigureMotors() {
        applySubsystems((s) -> s.applyMotorConfigs());
    }
    
    public Command shoot(){
        return Commands.parallel(
            indexer.setDesiredValueCommand(30),
            conveyor.setDesiredValueCommand(25)
        );
    }
    
    public Command stopShooting(){
        return Commands.parallel(
            indexer.setDesiredValueCommand(0),
            conveyor.setDesiredValueCommand(0)
        );
    }

    public Command spinUpFlywheel(){
        return Commands.parallel(
            counterRoller.setDesiredValueCommand(30),
            shooterLeft.setDesiredValueCommand(50),
            shooterRight.setDesiredValueCommand(50)
        );
    }
        
    public Command stopFlywheel(){
        return Commands.parallel(
            counterRoller.setDesiredValueCommand(0),
            shooterLeft.setDesiredValueCommand(0),
            shooterRight.setDesiredValueCommand(0)
        );
    }

    public Command intakeDown(){
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(0)
        );
    }

    public Command intakeUp(){
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(3.8)
        );
    }

    public Command intakeSlapdownStop() {
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(0.0)
        );
    }

    public Command intake() {
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(35)
        );
    }

    public Command stopIntaking(){
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(0)
        );
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        applySubsystems((s) -> s.setNeutralMode(neutralMode));
    }

    /**
     * fully stops all subsystems by putting them into neutral and disabling them
     * subsystems do not reenable on their own
     * @return a command to stop
     */
    public Command stop() {
        return Commands.runOnce(() -> {
            applySubsystems((s) -> s.stop());
        });
    }   

    public void initialize() {
        applySubsystems((s) -> s.setEnabled(s.useSubsystem));
    }

    public void resetSubsystemValues() {
        applySubsystems((s) -> s.setDesiredValue(s.getDefaultValue()));
    }

    @Override
    public void initializeLogging() {
        applySubsystems((s) -> s.initializeLogging());
    }
}
