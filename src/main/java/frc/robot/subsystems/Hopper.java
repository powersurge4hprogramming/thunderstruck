package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANBus;

public class Hopper extends SubsystemBase {
    // =================================================================================================================
    // Private Data Members
    // =================================================================================================================
    private final TalonFX krakenX60;

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    public Hopper() {
        krakenX60 = new TalonFX(CANBus.ID.HOPPER.MOTOR, CANBus.BUS.RIO);
    }

    // -----------------------------------------------------------------------------------------------------------------
    public void unclasp() {
        krakenX60.set(0.5);
    }

    // -----------------------------------------------------------------------------------------------------------------
    public void stop() {
        krakenX60.set(0);
    }
}
