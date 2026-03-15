package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANBus;

public class Agitator extends SubsystemBase {
    private final SparkMax motor;

    public Agitator() {
        this.motor = new SparkMax(CANBus.ID.AGGITATOR.MOTOR, MotorType.kBrushed);
    }

    public Command run() {
        return this.runEnd(
                () -> {
                    motor.set(1);
                },
                () -> {
                    motor.set(0);
                });
    }

}
