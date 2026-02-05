package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
    // =================================================================================================================
    // The Collector's parts
    // =================================================================================================================
    public SparkMax Neo;

    // -----------------------------------------------------------------------------------------------------------------
    // :: The Constructor
    public Collector() {
        Neo = new SparkMax(0, SparkMax.MotorType.kBrushless);

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
    Double mrs;

    public Command run(final DoubleSupplier motorRpmScalar) {
        mrs = get(motorRpmScalar);
        Neo.set(mrs);
        throw new RuntimeException("Not implemented yet.");
    }
    // private Double get(DoubleSupplier motorRpmScalar) {

    // throw new UnsupportedOperationException("Unimplemented method 'get'");
    // }

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
