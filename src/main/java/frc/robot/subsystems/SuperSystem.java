package frc.robot.subsystems;

import static frc.robot.Constants.Subsystems.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SuperSystem implements Reportable {
    public NerdDrivetrain swerveDrivetrain;

    public SuperSystem(NerdDrivetrain swerveDrivetrain) {
        this.swerveDrivetrain = swerveDrivetrain;
    }
    
    // ------------------------------------ subsystems ------------------------------------ //
    public void reConfigureMotors() {
        intakeSlapdown  .applyMotorConfigs();
        intakeRoller    .applyMotorConfigs();
        conveyor        .applyMotorConfigs();
        indexer         .applyMotorConfigs();
        counterRoller   .applyMotorConfigs();
        shooter         .applyMotorConfigs();
    }
    
    public Command shoot(){
        return Commands.parallel(
            indexer.setDesiredValueCommand(20)
            );
    }
    
    public Command stopShooting(){
        return Commands.parallel(
            indexer.setDesiredValueCommand(0)
            );
    }

    public Command spinUpFlywheel(){
        return Commands.parallel(
            counterRoller.setDesiredValueCommand(30),
            shooter.setDesiredValueCommand(75)
            );
        }
        
    public Command stopFlywheel(){
        return Commands.parallel(
            counterRoller.setDesiredValueCommand(0),
            shooter.setDesiredValueCommand(0)
        );
    }

    public Command intakeDown(){
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(0)
        );
    }

    public Command intakeUp(){
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(6.5)
        );
    }

    public Command intakeSlapdownStop() {
        return Commands.parallel(
            intakeSlapdown.setDesiredValueCommand(0.0)
        );
    }

    public Command intake() {
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(4.5)
        );
    }

    public Command stopIntaking(){
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(0)
        );
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        intakeSlapdown  .setNeutralMode(neutralMode);
        intakeRoller    .setNeutralMode(neutralMode);
        conveyor        .setNeutralMode(neutralMode);
        indexer         .setNeutralMode(neutralMode);
        counterRoller   .setNeutralMode(neutralMode);
        shooter         .setNeutralMode(neutralMode);
    }

    /**
     * fully stops all subsystems by putting them into neutral and disabling them
     * subsystems do not reenable on their own
     * @return a command to stop
     */
    public Command stop() {
        return Commands.runOnce(() -> {
            intakeSlapdown  .stop();
            intakeRoller    .stop();
            conveyor        .stop();
            indexer         .stop();
            counterRoller   .stop();
            shooter         .stop();
        });
    }   

    public void initialize() {
        intakeSlapdown  .setEnabled(useIntakeSlapdown);
        intakeRoller    .setEnabled(useIntakeRoller);
        conveyor        .setEnabled(useConveyor);
        indexer         .setEnabled(useIndexer);
        counterRoller   .setEnabled(useCounterRoller);
        shooter         .setEnabled(useShooter);
    }

    @Override
    public void initializeLogging() {
        intakeSlapdown  .initializeLogging();
        intakeRoller    .initializeLogging();
        conveyor        .initializeLogging();
        indexer         .initializeLogging();
        counterRoller   .initializeLogging();
        shooter         .initializeLogging();
        // just imagine it
        // applyToSubsystems((subsystem) -> subsystem.initializeLogging());
    }

    // /**
    //  * i really wanna use this
    //  * its so tempting
    //  * @param f
    //  */
    // public void applyToSubsystems(Consumer<TemplateSubsystem> f) {
    //     Field[] fields = Subsystems.class.getFields();
    //     for (int i = 0; i < fields.length; i++) {
    //         if (fields[i].getType().equals(TemplateSubsystem.class)) {
    //             try {
    //                 f.accept((TemplateSubsystem)fields[i].get(null));
    //             } catch (IllegalArgumentException e) {
    //                 e.printStackTrace();
    //             } catch (IllegalAccessException e) {
    //                 e.printStackTrace();
    //             }
    //         }
    //     }

    //     // ---- or ----
    //     f.accept(intakeSlapdown);
    //     f.accept(intakeRoller);
    //     f.accept(conveyor);
    //     f.accept(indexer);
    //     f.accept(counterRoller);
    //     f.accept(shooter);
    // }
}
