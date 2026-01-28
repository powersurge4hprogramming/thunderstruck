package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.CANBus;

public class Shooter extends SubsystemBase {
    private final TalonFX motorLeader;
    private final TalonFX motorFollower;

    public Shooter() {
        this.motorLeader = new TalonFX(CANBus.ID.SHOOTER.LEADER, CANBus.BUS.CANIVORE);
        this.motorFollower = new TalonFX(CANBus.ID.SHOOTER.FOLLOWER, CANBus.BUS.CANIVORE);

        this.motorFollower.setControl(new Follower(this.motorLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    /**
     * This function creates a single {@link Command} to shoot the ball at a
     * particular instant. It will shoot the ball at any given velocity and launch
     * angle.
     * 
     * @param motorRPMScalar The "speed" at which to set the motor in percentage of
     *                       possible RPM from 0 to 1.
     * @param angleLaunch    The angle at which to shoot the ball.
     * 
     * @return The {@link Command} at which to shoot the ball.
     */
    public Command manualShootBall(final DoubleSupplier motorRPMScalar, final DoubleSupplier angleLaunch) {
        throw new RuntimeException("Not implemented yet.");
    }

    /**
     * {@summary}
     * This method sets the Shooter's motors to an exact RPM.
     * 
     * @apiNote
     *          This should only be used if you know what you're doing.
     * 
     * @param rpm The motor RPM.
     */
    public void setRPM(final double rpm) {
        throw new RuntimeException("Not implemented yet.");
    }

    /**
     * {@summary}
     * This method sets the Shooter's launch angle.
     * 
     * @apiNote
     *          This should only be used if you know what you're doing.
     * 
     * @param launchAngleDeg The angle, in degrees, at which to set the shooter.
     */
    public void setLaunchAngle(final double launchAngleDeg) {
        throw new RuntimeException("Not implemented yet.");
    }

    /**
     * {@summary}
     * The current RPM of the leader motor.
     * 
     * @return The motor's RPM.
     */
    public double getMotorRPM() {
        return this.motorLeader.getVelocity().getValue().in(RPM);
    }
}
