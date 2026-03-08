package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANBus;

public class Loader extends SubsystemBase {
    // =================================================================================================================
    // Private Data Members
    // =================================================================================================================
    private final TalonFX loader;

    public Loader() {
        this.loader = new TalonFX(CANBus.ID.SHOOTER.LOADER, CANBus.BUS.RIO);
    }

    // -----------------------------------------------------------------------------------------------------------------
    public Command manualLoaderRun() {
        return this.runEnd(
                // Run continuously while the command is active
                () -> {
                    final double loaderMotorSpeed = 0.5;
                    setLoaderSpeed(loaderMotorSpeed);
                },
                // Cleanup when the command ends (button released)
                () -> {
                    setLoaderSpeed(0);
                });
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * {@summary}
     * Sets the loader motor speed.
     *
     * @param speed The output to the loader motor from -1.0 to 1.0.
     *              Positive values feed the note into the shooter.
     */
    public void setLoaderSpeed(final double speed) {
        this.loader.set(speed);
    }
}
