package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANBus;

public class Collector extends SubsystemBase {
    // =================================================================================================================
    // Private Data Members
    // =================================================================================================================
    private TalonFX krakenX60;

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    public Collector() {
        krakenX60 = new TalonFX(CANBus.ID.COLLECTOR.MOTOR, CANBus.BUS.RIO);
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * {@summary}
     * Run the collector.
     *
     * @param motorRpmScalar A value from 0 to 1 that represents the percentage at
     *                       which to run the motor.
     * 
     * @return The {@link Command} to perform the action.
     */
    public Command run(final DoubleSupplier motorRpmScalar) {
        return this.run(() -> {
            double mrs = motorRpmScalar.getAsDouble();
            krakenX60.set(mrs);
        }).handleInterrupt(() -> {
            krakenX60.set(0);
        });
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * {@summary}
     * Stop the collector.
     * 
     * @return The {@link Command} to perform the action.
     */
    public Command stop() {
        throw new RuntimeException("Not implemented yet.");
    }
}
