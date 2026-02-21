package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.NeutralOut;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.CANBus;

public class Shooter extends SubsystemBase {
    private final double MAX_SHOOTER_RPM = 5_500.0; // <- Tune this after characterization

    private final TalonFX motorLeader;
    private final TalonFX motorFollower;
    private final SparkMax loader;
    private final VelocityVoltage velocityRequest;

    public Shooter() {
        this.motorLeader = new TalonFX(CANBus.ID.SHOOTER.LEADER, CANBus.BUS.CANIVORE);
        this.motorFollower = new TalonFX(CANBus.ID.SHOOTER.FOLLOWER, CANBus.BUS.CANIVORE);
        this.loader = new SparkMax(CANBus.ID.SHOOTER.LOADER, SparkMax.MotorType.kBrushless);

        this.motorFollower.setControl(new Follower(this.motorLeader.getDeviceID(), MotorAlignmentValue.Aligned));
        this.velocityRequest = new VelocityVoltage(0).withSlot(0);

        configureMotors();
    }

    /**
     * Configures both shooter motors (PIDF, current limits, neutral mode).
     * Called once from the constructor.
     */
    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor output & safety
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Most teams prefer Coast for shooters
        // config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Uncomment if you
        // want brake

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

    /**
     * This function creates a single {@link Command} to shoot the ball at a
     * particular instant. It will shoot the ball at any given velocity and launch
     * angle.
     * 
     * @param motorRPMScalar   The "speed" at which to set the shoot motor in
     *                         percentage of possible RPM from 0 to 1.
     * @param loaderMotorOnOff The toggle for the loader to turn on or off.
     * 
     * @return The {@link Command} at which to shoot the ball.
     */
    public Command manualShootBall(final DoubleSupplier motorRPMScalar, final BooleanSupplier loaderMotorOnOff) {
        final double loaderMotorSpeed = 0.8;
        return this.runEnd(
                // Run continuously while the command is active
                () -> {
                    double targetRPM = motorRPMScalar.getAsDouble() * MAX_SHOOTER_RPM;
                    setRPM(targetRPM);

                    if (loaderMotorOnOff.getAsBoolean()) {
                        setLoaderSpeed(loaderMotorSpeed);
                    } else {
                        setLoaderSpeed(0.0);
                    }
                },
                // Cleanup when the command ends (button released)
                () -> {
                    stopShooter();
                    setLoaderSpeed(0.0);
                });
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
        if (rpm < 100) { // Safety - don't try to spin at tiny speeds
            stopShooter();
            return;
        }

        final double targetRPS = rpm / 60.0; // TalonFX uses Rotations Per Second
        velocityRequest.Velocity = targetRPS;

        motorLeader.setControl(velocityRequest);
    }

    /**
     * Stops the shooter motors.
     */
    public void stopShooter() {
        motorLeader.setControl(new NeutralOut());
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

    /**
     * {@summary}
     * Sets the loader motor speed.
     *
     * @param speed The output to the loader motor from -1.0 to 1.0.
     *              Positive values feed the note into the shooter.
     */
    public void setLoaderSpeed(final double speed) {
        this.loader.set(speed);
    }
}
