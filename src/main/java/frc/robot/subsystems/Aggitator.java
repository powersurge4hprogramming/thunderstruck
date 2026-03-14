package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Aggitator extends SubsystemBase {
    private final SparkMax motor;

    public Aggitator() {
        this.motor = new SparkMax(0, MotorType.kBrushed);
    }

}
