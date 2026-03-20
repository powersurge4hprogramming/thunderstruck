package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANBus;

public class Collector extends SubsystemBase {
    // =================================================================================================================
    // Private Data Members
    // =================================================================================================================
    private TalonFX krakenX60;
    private SparkMax motor;

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    public Collector() {
        krakenX60 = new TalonFX(CANBus.ID.COLLECTOR.ENTRY_MOTOR, CANBus.BUS.RIO);
        motor = new SparkMax(CANBus.ID.COLLECTOR.CONVEYOR, MotorType.kBrushed);
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
        return this.runEnd(
                () -> {
                    double mrs = motorRpmScalar.getAsDouble() * 0.5;
                    krakenX60.set(mrs);
                    if (mrs == 0)
                        motor.set(0);
                    else
                        motor.set(-1);
                },
                () -> {
                    krakenX60.set(0);
                    motor.set(0);
                });
    }

}
