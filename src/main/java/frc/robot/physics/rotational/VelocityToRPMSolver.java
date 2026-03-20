package frc.robot.physics.rotational;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;

/**
 * Physics-based flywheel RPM solver for FRC RoboRIO 2 (WPILib 2026).
 * <p>
 * Converts a desired ball exit velocity (m/s) from the ballistic solver
 * into a motor RPM setpoint for the TalonFX / Kraken X60 flywheel.
 * Also provides a hysteresis-based readiness check for feeding.
 *
 * <h3>Core physics model:</h3>
 * 
 * <pre>
 *   v_exit = ω_wheel · r · η
 *
 *   where:
 *     v_exit  = ball exit velocity (m/s)
 *     ω_wheel = wheel angular speed (rad/s)
 *     r       = wheel radius (m)
 *     η       = exit velocity efficiency (dimensionless, 0–1)
 *
 *   Rearranged:
 *     ω_wheel = v_exit / (r · η)
 *
 *   Converting to motor RPM through gear ratio G:
 *     RPM_motor = ω_wheel · G · 60 / (2π)
 * </pre>
 *
 * <h3>Numeric worked example (using actual class constants):</h3>
 * 
 * <pre>
 *   Given: v_exit = 10.0 m/s
 *
 *   ω_wheel = 10.0 / (0.0508 × 0.60)
 *           = 10.0 / 0.03048
 *           ≈ 328.08 rad/s
 *
 *   RPM_motor = 328.08 × 1.0 × 60 / (2π)
 *             = 328.08 × 9.5493
 *             ≈ 3133 RPM
 *
 *   Max motor speed at 12 V:
 *     RPM_max = KV × V_NOM = 500 × 12 = 6000 RPM
 *     3133 &lt; 6000 ✓  (not clamped)
 * </pre>
 *
 * <h3>Pit-test checklist for measuring/tuning constants:</h3>
 * <ul>
 * <li><b>WHEEL_RADIUS_M</b>: Measure physical wheel radius with calipers
 * or from CAD.</li>
 * <li><b>EXIT_VELOCITY_EFFICIENCY</b>: Fire shots at various known RPM
 * (measured by encoder), measure exit velocity with chronograph,
 * fit η = measured_v / (ω_wheel × radius).</li>
 * <li><b>READY_TOLERANCE_RPM</b>: Tune based on acceptable velocity error
 * (e.g., 100 RPM ≈ 3% at 3000 RPM).</li>
 * <li><b>MIN_READY_TIME_S</b>: Tune to filter noise/oscillations
 * (start at 0.1 s).</li>
 * <li><b>KV_RPM_PER_V</b>: From motor datasheet (Kraken X60 ≈ 500 RPM/V,
 * confirm via CTRE docs for your specific firmware).</li>
 * </ul>
 *
 * <h3>Unit-test plan:</h3>
 * <ul>
 * <li>Linear calc: calculateMotorRPM(10.0) → verify ≈ 3133 RPM.</li>
 * <li>Zero input: calculateMotorRPM(0.0) → 0.0 RPM.</li>
 * <li>Negative input: calculateMotorRPM(-5.0) → 0.0 RPM.</li>
 * <li>Readiness: mock measuredRPM near target, verify true after
 * hysteresis time; mock far from target, verify false.</li>
 * <li>Feedforward: getKF(12.0) ≈ 0.002 V/RPM.</li>
 * </ul>
 *
 * <h3>PID + Feedforward Tuning Guidance:</h3>
 * <ul>
 * <li>Tune kF/kV first in open-loop (kP/kI/kD = 0): command RPM, measure
 * steady-state; set kF ≈ 1 / (KV × V_bat/V_NOM). Use SysId for kV/kA.</li>
 * <li>Add kP low (e.g., 0.0001 V/RPM error), reduce if oscillating.</li>
 * <li>Minimal kI (e.g., 0.00001) for steady-state error; avoid windup.</li>
 * <li>kD small for damping oscillation.</li>
 * <li>Use CTRE Phoenix Tuner X for live tuning on Kraken X60 / Talon FX.</li>
 * </ul>
 */
public class VelocityToRPMSolver {

    // =========================================================================
    // Measured / Tunable Constants
    // =========================================================================

    /** Flywheel wheel radius (m). Measure with calipers or from CAD. */
    private static final double WHEEL_RADIUS_M = 0.0508; // 2 inches

    /**
     * Fraction of wheel surface speed that becomes ball exit speed.
     * 
     * <pre>
     *   η = v_ball / (ω_wheel · r)
     * </pre>
     * 
     * Losses come from slip, compression, friction. Measure empirically.
     */
    private static final double EXIT_VELOCITY_EFFICIENCY = 0.60;

    /** Acceptable RPM error band for the readiness check. */
    private static final double READY_TOLERANCE_RPM = 100.0;

    /**
     * Minimum continuous time (seconds) that measured RPM must stay
     * within tolerance before {@link #isReadyToFire()} returns true.
     * Prevents firing during transient oscillations.
     */
    private static final double MIN_READY_TIME_S = 0.1;

    // =========================================================================
    // Motor Datasheet Constants
    // =========================================================================

    /** Nominal battery voltage (V). Used for feedforward scaling. */
    private static final double V_NOM = 12.0;

    /**
     * Motor velocity constant (RPM per Volt, no-load).
     * <p>
     * Kraken X60 ≈ 500 RPM/V (confirm from CTRE documentation for
     * your firmware version). Free speed at 12 V ≈ 6000 RPM.
     */
    private static final double KV_RPM_PER_V = 500.0;

    /**
     * Gear ratio: motor turns per wheel turn.
     * 
     * <pre>
     *   G = 1.0 → direct drive (no gearbox)
     *   G = 2.0 → motor spins twice per wheel revolution
     * </pre>
     */
    private static final double G_GEAR_RATIO = 1.0;

    // =========================================================================
    // Precomputed Coefficients (avoid recomputing every call)
    // =========================================================================

    /**
     * Converts wheel rad/s → motor RPM.
     * 
     * <pre>
     *   RPM_motor = ω_wheel · G · 60 / (2π)
     *             = ω_wheel · OMEGA_TO_RPM_FACTOR
     * </pre>
     */
    private final double OMEGA_TO_RPM_FACTOR = G_GEAR_RATIO * 60.0 / (2.0 * Math.PI);

    /**
     * Converts ball exit m/s → wheel rad/s.
     * 
     * <pre>
     *   ω_wheel = v_exit / (r · η)
     *           = v_exit · V_TO_OMEGA_FACTOR
     * </pre>
     */
    private final double V_TO_OMEGA_FACTOR = 1.0 / (WHEEL_RADIUS_M * EXIT_VELOCITY_EFFICIENCY);

    // =========================================================================
    // State Variables
    // =========================================================================

    /** Most recent target wheel angular speed (rad/s). */
    private double targetWheelOmega = 0.0;

    /** Most recent target motor RPM setpoint. */
    private double targetMotorRPM = 0.0;

    /**
     * FPGA timestamp (seconds) when the RPM error first dropped below
     * tolerance. Reset to −1 when error exceeds tolerance.
     */
    private double lastReadyTime = -1.0;

    /** Whether the previous call to isReadyToFire() found error in-band. */
    private boolean wasReadyLastCall = false;

    // =========================================================================
    // Suppliers
    // =========================================================================

    /** Provides the current measured motor RPM from the TalonFX encoder. */
    private final DoubleSupplier measuredMotorRPMSupplier;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * @param measuredMotorRPMSupplier Supplier returning the current motor
     *                                 RPM as read from the TalonFX encoder.
     *                                 Example: {@code () -> shooter.getMotorRPM()}
     */
    public VelocityToRPMSolver(DoubleSupplier measuredMotorRPMSupplier) {
        this.measuredMotorRPMSupplier = measuredMotorRPMSupplier;
    }

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * Computes the motor RPM setpoint for a desired ball exit velocity.
     *
     * <h3>Math:</h3>
     * 
     * <pre>
     *   ω_wheel    = v_exit / (r · η)
     *   RPM_motor  = ω_wheel · G · 60 / (2π)
     * </pre>
     *
     * @param desiredExitVelocityMps Desired ball exit speed (m/s), from
     *                               {@code ShotResult.getFlywheelSpeedMPS()}.
     * @return Motor RPM setpoint. Returns 0 for non-positive input.
     */
    public double calculateMotorRPM(double desiredExitVelocityMps) {
        if (desiredExitVelocityMps <= 0.0) {
            targetWheelOmega = 0.0;
            targetMotorRPM = 0.0;
            return 0.0;
        }

        // v_exit → ω_wheel (rad/s)
        targetWheelOmega = desiredExitVelocityMps * V_TO_OMEGA_FACTOR;

        // ω_wheel → motor RPM (through gear ratio)
        targetMotorRPM = targetWheelOmega * OMEGA_TO_RPM_FACTOR;
        return targetMotorRPM;
    }

    /**
     * Checks if the flywheel is ready to fire.
     * <p>
     * Returns {@code true} only when <b>both</b> conditions hold:
     * <ol>
     * <li>|measured RPM − target RPM| &lt; READY_TOLERANCE_RPM</li>
     * <li>The above has been true continuously for ≥ MIN_READY_TIME_S</li>
     * </ol>
     * This two-stage check prevents firing during transient speed
     * oscillations from PID overshoot or ball loading disturbances.
     * <p>
     * Call this every loop iteration <b>after</b> {@link #calculateMotorRPM}.
     *
     * @return true if the flywheel is at speed and stable.
     */
    public boolean isReadyToFire() {
        // [FIX #1] Removed System.out.println that ran every frame.
        // String concatenation allocates on the heap → GC pauses.

        if (targetMotorRPM <= 0.0) {
            lastReadyTime = -1.0;
            wasReadyLastCall = false;
            return false;
        }

        // [FIX #6] Use WPILib's FPGA timer instead of System.nanoTime().
        //
        // System.nanoTime():
        // • Works on real hardware but is NOT simulated by WPILib.
        // • long → double conversion loses precision at large uptimes.
        //
        // Timer.getFPGATimestamp():
        // • Returns seconds since FPGA power-on (double, µs precision).
        // • Correctly paused/advanced in WPILib simulation and replay.
        // • The canonical clock for all FRC timing.
        double now = Timer.getFPGATimestamp();
        double measuredRPM = measuredMotorRPMSupplier.getAsDouble();
        double error = Math.abs(measuredRPM - targetMotorRPM);
        boolean currentlyReady = error < READY_TOLERANCE_RPM;

        if (currentlyReady) {
            if (!wasReadyLastCall) {
                // Transition: out-of-band → in-band. Start the clock.
                lastReadyTime = now;
            }
            wasReadyLastCall = true;

            // Check if we've been in-band long enough
            if (now - lastReadyTime >= MIN_READY_TIME_S) {
                return true;
            }
        } else {
            // Out of band — reset hysteresis
            lastReadyTime = -1.0;
            wasReadyLastCall = false;
        }
        return false;
    }

    /**
     * Computes a simple feedforward gain kF (Volts per RPM).
     *
     * <h3>Math:</h3>
     * 
     * <pre>
     *   Motor: RPM_free = KV · V_applied
     *   So:    V_needed  = RPM_target / KV
     *
     *   With voltage compensation for battery sag:
     *     kF = V_NOM / (KV · V_bat)
     *
     *   Feedforward voltage: V_ff = kF · RPM_target
     * </pre>
     *
     * <h3>Example:</h3>
     * 
     * <pre>
     *   KV = 500 RPM/V, V_bat = 12 V:
     *   kF = 12 / (500 × 12) = 0.002 V/RPM
     *
     *   For 3000 RPM target: V_ff = 0.002 × 3000 = 6.0 V  ✓
     *   (3000 RPM / 500 RPM/V = 6.0 V — checks out)
     * </pre>
     *
     * <h3>CTRE integration note:</h3>
     * <p>
     * If using TalonFX voltage-compensated control, you can convert this
     * to CTRE's kV (Volts per RPS) by multiplying by 60:
     * 
     * <pre>
     *   CTRE_kV = getKF(vbat) × 60    (V/RPM × 60 s/min = V/RPS)
     * </pre>
     *
     * @param currentBatteryVoltage Current battery voltage (V).
     * @return kF in Volts per motor RPM.
     */
    public double getKF(double currentBatteryVoltage) {
        if (currentBatteryVoltage <= 0.0)
            return 0.0;
        return V_NOM / (KV_RPM_PER_V * currentBatteryVoltage);
    }

    // =========================================================================
    // Telemetry Getters (read-only, for SmartDashboard / DataLog)
    // =========================================================================

    /** Target wheel angular speed (rad/s). */
    public double getTargetWheelOmega() {
        return targetWheelOmega;
    }

    /** Target motor RPM setpoint. */
    public double getTargetMotorRPM() {
        return targetMotorRPM;
    }

    /** Current measured motor RPM from encoder. */
    public double getMeasuredMotorRPM() {
        return measuredMotorRPMSupplier.getAsDouble();
    }
}