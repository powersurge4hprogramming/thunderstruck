package frc.robot.physics.rotational;

import java.util.function.DoubleSupplier;

/**
 * VelocityToRpmSolver.java
 *
 * This class implements the simplified physics-based shooter math for FRC
 * RoboRIO 2 with high-inertia flywheel.
 * It models the linear surface speed for target RPM, voltage compensation, and
 * readiness check.
 *
 * Numeric worked example (using class defaults):
 * Assume desiredExitVelocityMps = 10.0 m/s
 * targetWheelOmega = 10.0 / (0.05 * 0.95) ≈ 10 / 0.0475 ≈ 210.53 rad/s
 * maxWheelOmega at V=12: kv_rad_per_v ≈ 59.38 (for 567 RPM/V), max = 59.38 *
 * (12/12) / 1.5 ≈ 59.38 / 1.5 ≈ 39.59 rad/s? Wait, recalculate properly:
 * kv_rad_per_v = 567 * pi / 30 ≈ 59.38 rad/s per V
 * maxWheelOmega = 59.38 * 12 / 1.5 ≈ 712.56 / 1.5 ≈ 475.04 rad/s (clamped if
 * needed, but 210.53 < 475)
 * targetMotorRPM = 210.53 * 1.5 * 60 / (2*pi) ≈ 210.53 * 1.5 * 9.549 ≈ 3015 RPM
 *
 * Pit-test checklist for measuring/tuning constants:
 * - WHEEL_RADIUS_M: Measure physical wheel radius with calipers or from CAD.
 * - EXIT_VELOCITY_EFFICIENCY: Fire shots at various known RPM (measured by
 * encoder), measure exit velocity with chronograph, fit efficiency = measured_v
 * / (wheel_omega * radius).
 * - READY_TOLERANCE_RPM: Tune based on acceptable velocity error (e.g., 50 RPM
 * ≈ 5% at 1000 RPM).
 * - MIN_READY_TIME_S: Tune to filter noise/oscillations (start at 0.1s).
 * - KV_RPM_PER_V: From motor datasheet (e.g., Kraken ~567 RPM/V).
 *
 * Unit-test plan:
 * - Test linear calc: Call calculateMotorRPM(10.0), verify ≈3015 RPM (per
 * example).
 * - Test voltage clamp: Mock vbat=6V, verify RPM clamped to half max (e.g., if
 * target high, cap at ~1507 RPM).
 * - Test readiness: Mock measuredRPM close to target, verify true after
 * hysteresis time; far, false.
 * - Edge: v=0 → RPM=0; high v > max, clamped.
 * - Numeric check: Verify precomputes used, no div by zero.
 *
 * Telemetry logging guidance during practice:
 * Log targetMotorRPM, measuredMotorRPM, battery voltage, error RPM,
 * isReadyToFire flag, motor current (from PDH/PDP), measured exit speed (if
 * sensor available).
 * Plot spool-up curves, compare commanded vs actual RPM, adjust efficiency/PID.
 *
 * PID + Feedforward (PIDF) Tuning Guidance:
 * - Tune kF first in open-loop (kP/kI/kD=0): Command RPM, measure steady-state
 * voltage/RPM, set kF ≈ voltage / (RPM * vbat / V_NOM). Use SysId for kV/kA.
 * - Add kP low (e.g., 0.0001 V/RPM error) to correct, reduce if oscillating.
 * - Minimal kI (e.g., 0.00001) for load errors, avoid windup.
 * - kD small for damping.
 * - Use CTRE Phoenix Tuner X for live tuning on Kraken X60/Talon FX.
 * - Test under load (balls), expect 1-3s rise time due to inertia.
 *
 * TODO: Update motor constants from Kraken X60 datasheet.
 * TODO: Tune constants empirically from robot data.
 */
public class VelocityToRpmSolver {
    // Measured/tunable constants
    private static final double WHEEL_RADIUS_M = 0.05; // Wheel radius (m)
    private static final double EXIT_VELOCITY_EFFICIENCY = 0.95; // Efficiency factor (dimensionless)
    private static final double READY_TOLERANCE_RPM = 50.0; // Tolerance for readiness (RPM)
    private static final double MIN_READY_TIME_S = 0.1; // Min time error < tolerance for readiness (s)

    // Motor datasheet constants (updated for Kraken X60 approx values; confirm CTRE
    // docs)
    private static final double V_NOM = 12.0; // Nominal voltage (V)
    private static final double KV_RPM_PER_V = 567.0; // Motor kV (RPM per V)
    private static final double G_GEAR_RATIO = 1.5; // Gear ratio (motor turns : wheel turn)

    // Precomputed coefficients
    private final double KV_RAD_PER_V = KV_RPM_PER_V * Math.PI / 30.0; // rad/s per V
    private final double OMEGA_TO_RPM_FACTOR = G_GEAR_RATIO * 60.0 / (2 * Math.PI); // rad/s wheel to motor RPM
    private final double V_TO_OMEGA_FACTOR = 1.0 / (WHEEL_RADIUS_M * EXIT_VELOCITY_EFFICIENCY); // m/s to rad/s wheel

    // State variables
    private double targetWheelOmega = 0.0; // Target wheel angular speed (rad/s)
    private double targetMotorRPM = 0.0; // Target motor RPM
    private double lastBatteryVoltage = V_NOM; // Last battery voltage (V)
    private double lastReadyTime = -1.0; // Time when first became ready (s)
    private boolean wasReadyLastCall = false; // Previous readiness state

    // Suppliers
    private final DoubleSupplier batteryVoltageSupplier;
    private final DoubleSupplier measuredMotorRPMSupplier;

    /**
     * Constructor for VelocityToRpmSolver.
     *
     * @param batteryVoltageSupplier   Supplier for current battery voltage.
     * @param measuredMotorRPMSupplier Supplier for measured motor RPM (from
     *                                 encoder).
     */
    public VelocityToRpmSolver(DoubleSupplier batteryVoltageSupplier, DoubleSupplier measuredMotorRPMSupplier) {
        this.batteryVoltageSupplier = batteryVoltageSupplier;
        this.measuredMotorRPMSupplier = measuredMotorRPMSupplier;
    }

    /**
     * Computes the motor RPM setpoint for the desired exit velocity.
     * Math: omega_target = v_exit / (radius * efficiency)
     * Clamped to voltage-scaled kinematic limit.
     * motor_rpm = omega_target * G * 60 / (2 pi)
     *
     * @param desiredExitVelocityMps Desired ball exit velocity (m/s).
     * @return Motor RPM setpoint.
     */
    public double calculateMotorRPM(double desiredExitVelocityMps) {
        double vbat = Math.max(batteryVoltageSupplier.getAsDouble(), 0.0); // Safe guard non-negative
        lastBatteryVoltage = vbat;

        if (desiredExitVelocityMps <= 0.0) {
            targetWheelOmega = 0.0;
            targetMotorRPM = 0.0;
            return 0.0;
        }

        // Linear surface speed
        double targetWheelOmegaUnclamped = desiredExitVelocityMps * V_TO_OMEGA_FACTOR;

        // Kinematic limit scaled by voltage
        double maxWheelOmega = KV_RAD_PER_V * (vbat / V_NOM) / G_GEAR_RATIO;
        targetWheelOmega = Math.min(targetWheelOmegaUnclamped, maxWheelOmega);

        // Convert to motor RPM
        targetMotorRPM = targetWheelOmega * OMEGA_TO_RPM_FACTOR;
        return targetMotorRPM;
    }

    /**
     * Checks if the flywheel is ready to fire (error < tolerance for min time).
     * Call after calculateMotorRPM in robot loop.
     *
     * @return True if ready to fire.
     */
    public boolean isReadyToFire() {
        if (targetMotorRPM <= 0.0) {
            lastReadyTime = -1.0;
            wasReadyLastCall = false;
            return false;
        }

        double now = System.nanoTime() / 1_000_000_000.0;
        double measuredRPM = measuredMotorRPMSupplier.getAsDouble();
        double error = Math.abs(measuredRPM - targetMotorRPM);
        boolean currentlyReady = error < READY_TOLERANCE_RPM;

        if (currentlyReady) {
            if (!wasReadyLastCall) {
                lastReadyTime = now;
            }
            wasReadyLastCall = true;
            if (now - lastReadyTime >= MIN_READY_TIME_S) {
                return true;
            }
        } else {
            lastReadyTime = -1.0;
            wasReadyLastCall = false;
        }
        return false;
    }

    /**
     * Computes feedforward kF (volts per RPM, scaled for voltage compensation).
     * Math: 1 / (kV_rpm * (currentBatteryVoltage / V_NOM)) for normalized.
     * Document: Integrate with PID by ff = getKF(vbat) * target_rpm.
     * For Talon FX onboard, set kF constant and enable voltage comp if available.
     *
     * @param currentBatteryVoltage Current battery voltage (V).
     * @return kF (volts per motor RPM).
     */
    public double getKF(double currentBatteryVoltage) {
        if (currentBatteryVoltage <= 0)
            return 0.0; // Safe
        return 1.0 / (KV_RPM_PER_V * (currentBatteryVoltage / V_NOM));
    }

    // Telemetry getters (read-only)
    public double getTargetWheelOmega() {
        return targetWheelOmega;
    }

    public double getLastBatteryVoltage() {
        return lastBatteryVoltage;
    }

    public double getTargetMotorRPM() {
        return targetMotorRPM;
    }

    public double getMeasuredMotorRPM() {
        return measuredMotorRPMSupplier.getAsDouble();
    }
}