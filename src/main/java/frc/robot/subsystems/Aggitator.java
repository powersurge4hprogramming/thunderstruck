package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANBus;

public class Aggitator extends SubsystemBase {
    private final SparkMax motor;

    public Aggitator() {
        this.motor = new SparkMax(CANBus.ID.AGGITATOR.MOTOR, MotorType.kBrushed);
    }

    public Command run(final DoubleSupplier motorRpmScalar) {
        return this.runEnd(
                () -> {
                    motor.set(motorRpmScalar.getAsDouble());
                },
                () -> {
                    motor.set(0);
                });
    }

}
