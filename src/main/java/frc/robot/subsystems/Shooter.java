package frc.robot.subsystems;

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
     * This function creates a single command to shoot the ball at a particular
     * instant. It will shoot the ball at any
     * given velocity and launch angle
     * 
     * @param velocityBall The velocity at which to shoot the ball.
     * @param angleLaunch  The angle at which to shoot the ball.
     * 
     * @return The {@link Command} at which to shoot the ball.
     */
    public Command shootBall(final double velocityBall, final double angleLaunch) {
        throw new RuntimeException("Not implemented yet.");
    }
}