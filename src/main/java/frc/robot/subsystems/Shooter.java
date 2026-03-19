package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.CANBus;

public class Shooter extends SubsystemBase {
    // =================================================================================================================
    // Private Data Members
    // =================================================================================================================
    private final double MAX_SHOOTER_RPM = 5_000.0; // <- Tune this after characterization

    private final TalonFX motorLeader;
    private final TalonFX motorFollower;
    private final VelocityVoltage velocityRequest;

    private final NetworkTableEntry leaderRpmEntry;
    private final NetworkTableEntry askedRpm;

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    public Shooter() {
        this.motorLeader = new TalonFX(CANBus.ID.SHOOTER.LEADER, CANBus.BUS.CANIVORE);
        this.motorFollower = new TalonFX(CANBus.ID.SHOOTER.FOLLOWER, CANBus.BUS.CANIVORE);

        this.motorFollower.setControl(new Follower(this.motorLeader.getDeviceID(), MotorAlignmentValue.Opposed));
        this.velocityRequest = new VelocityVoltage(0).withSlot(0);

        configureMotors();

        NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("Shooter");
        this.leaderRpmEntry = shooterTable.getEntry("LeaderRPM");
        this.askedRpm = shooterTable.getEntry("Asked RPM");
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * Configures both shooter motors (PIDF, current limits, neutral mode).
     * Called once from the constructor.
     */
    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor output & safety
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Most teams prefer Coast for shooters
        // Uncomment if you want brake
        // config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Current limiting (important for Kraken X60)
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 100;

        // Slot 0 - Velocity Control (starting gains)
        config.Slot0.kP = 0.22;
        config.Slot0.kI = 0.0005; // Small I term helps hold speed
        config.Slot0.kD = 0.008;
        config.Slot0.kV = 0.115; // Feedforward - very important!
        config.Slot0.kS = 0.18; // Static friction compensation

        // Apply to both motors
        motorLeader.getConfigurator().apply(config);
        motorFollower.getConfigurator().apply(config);
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * This function creates a single {@link Command} to shoot the ball at a
     * particular instant. It will shoot the ball at any given velocity and launch
     * angle.
     * 
     * @param motorRPMScalar The "speed" at which to set the shoot motor in
     *                       percentage of possible RPM from 0 to 1.
     * 
     * @return The {@link Command} at which to shoot the ball.
     */
    public Command manualShootBall(final DoubleSupplier motorRPMScalar) {
        return this.runEnd(
                // Run continuously while the command is active
                () -> {
                    // double targetRPM = motorRPMScalar.getAsDouble() * MAX_SHOOTER_RPM;
                    double targetRPM = 0.54 * MAX_SHOOTER_RPM;
                    setRPM(targetRPM);

                    leaderRpmEntry.setDouble(motorLeader.getVelocity().getValueAsDouble() * 60.0);
                },
                // Cleanup when the command ends (button released)
                () -> {
                    stopShooter();
                    leaderRpmEntry.setDouble(motorLeader.getVelocity().getValueAsDouble() * 60.0);
                });
    }

    // -----------------------------------------------------------------------------------------------------------------
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
        askedRpm.setDouble(rpm);
        if (rpm < 100) { // Safety - don't try to spin at tiny speeds
            stopShooter();
            return;
        }

        final double targetRPS = rpm / 60.0; // TalonFX uses Rotations Per Second
        velocityRequest.Velocity = targetRPS;

        motorLeader.setControl(velocityRequest);
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * Stops the shooter motors.
     */
    public void stopShooter() {
        motorLeader.setControl(new NeutralOut());
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * {@summary}
     * The current RPM of the leader motor.
     * 
     * @return The motor's RPM.
     */
    public double getMotorRPM() {
        return this.motorLeader.getVelocity().getValue().in(RPM);
    }

    // -----------------------------------------------------------------------------------------------------------------
    public double getMaxRPM() {
        return MAX_SHOOTER_RPM;
    }

}
