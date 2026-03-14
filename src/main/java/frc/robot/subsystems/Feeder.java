package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANBus;

public class Feeder extends SubsystemBase {
    // =================================================================================================================
    // Private Data Members
    // =================================================================================================================
    private final TalonFX loader;

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    public Feeder() {
        this.loader = new TalonFX(CANBus.ID.FEEDER.MOTOR, CANBus.BUS.RIO);
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * This function creates a single {@link Command} to feed the ball at a
     * particular instant.
     * 
     * @return The {@link Command} at which to feed the ball to the shooter.
     */
    public Command manualFeederRun() {
        return this.runEnd(
                // Run continuously while the command is active
                () -> {
                    final double loaderMotorSpeed = 0.5;
                    setFeederSpeed(loaderMotorSpeed);
                },
                // Cleanup when the command ends (button released)
                () -> {
                    setFeederSpeed(0);
                });
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * {@summary}
     * Sets the feeder motor speed.
     *
     * @param speed The output to the feeder motor from -1.0 to 1.0.
     *              Positive values feed the ball into the shooter.
     * 
     * @apiNote
     *          Only use this if you know what you are doing.
     */
    public void setFeederSpeed(final double speed) {
        this.loader.set(speed);
    }
}
