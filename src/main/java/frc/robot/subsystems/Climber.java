package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.DigitalInputBus;

public class Climber extends SubsystemBase {
    private final double SPEED_SCALAR = 0.5;

    private final DigitalInput leftArmLimit;
    private final DigitalInput rightArmLimit;
    private final TalonFX krakenX60;

    // -----------------------------------------------------------------------------------------------------------------
    public Climber() {
        krakenX60 = new TalonFX(CANBus.ID.CLIMBER.MOTOR, CANBus.BUS.RIO);
        leftArmLimit = new DigitalInput(DigitalInputBus.CLIMBER.LEFT_ARM_LIMIT_SWITCH);
        rightArmLimit = new DigitalInput(DigitalInputBus.CLIMBER.RIGHT_ARM_LIMIT_SWITCH);
    }

    /**
     * {@summary}
     * Tell the climber to go upward.
     * 
     * @return The {@link Command} to perform the action.
     */
    public Command upward() {
        return this.run(() -> {
            final byte clockwise = 1;
            final byte counterClockwise = -1;
            double speed = krakenX60.get();
            if (speed == 0) {
                speed = clockwise;
            }
            if (leftArmLimit.get() || rightArmLimit.get()) {
                // Flip the sign, we hit a limit.
                speed = -speed;
            }
            krakenX60.set(speed * SPEED_SCALAR);
        })
        .handleInterrupt(() -> {
            krakenX60.set(0);
        });
    }

    /**
     * {@summary}
     * Tell the climber to go downward.
     * 
     * @return The {@link Command} to perform the action.
     */
    public Command downward() {
        return this.run(() -> {
            final byte counterClockwise = -1;
            final byte clockwise = 1;
            double speed = krakenX60.get();
            if (speed == 0 || speed = clockwise) {
                speed = counterClockwise;
            }
            if (leftArmLimit.get() || rightArmLimit.get()) {
                // Flip the sign, we hit a limit.
                speed = -speed;
            }
            krakenX60.set(speed * SPEED_SCALAR);
        })
        .handleInterrupt(() -> {
            krakenX60.set(0);
        });
    }
}
