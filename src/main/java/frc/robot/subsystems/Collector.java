package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
    // =================================================================================================================
    // The Collector's parts
    // =================================================================================================================
    sparkMax Neo = new SparkMax.CANBus.ID.collector.motor("brushless");

    // ------------------------------------------------------------------------------------------------------------------
    // :: The Constructor
    public Collector() {

    }

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
        // :: Delete me ;)
        throw new RuntimeException("Not implemented yet.");
    }

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
