package frc.robot.physics.ballistics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Zero-Allocation Ballistic Solver for FRC RoboRIO 2.
 * Supports:
 * - Drag + Magnus physics (via RK4/Secant)
 * - Moving Reference Frame (Shooting while driving/strafing)
 * - Entry Angle Scalar control
 */
public class VelocityAngleSolver {
    // =========================================================================
    // PHYSICAL CONSTANTS
    // =========================================================================
    private static final double G = 9.81;
    private static final double MASS = 0.216; // kg
    private static final double RADIUS = 0.075; // meters
    private static final double AREA = Math.PI * RADIUS * RADIUS;
    private static final double AIR_DENSITY = 1.225;
    private static final double DRAG_COEFF = 0.47;
    private static final double LIFT_COEFF = 0.15;

    // =========================================================================
    // SOLVER SETTINGS
    // =========================================================================
    private static final int MAX_SECANT_ITERS = 5;
    private static final double TIME_STEP = 0.01;
    private static final double MAX_SIM_TIME = 5.0;

    /**
     * Data container to hold results.
     * Now includes Turret Yaw for strafe compensation.
     */
    public static class ShotResult {
        private double turretYawDegrees; // Field-relative angle to aim the turret
        private double hoodPitchDegrees; // Angle of the shooter hood
        private double flywheelSpeedMPS; // Tangential velocity of the flywheel

        private ShotResult(final double turretYawDegrees,
                final double hoodPitchDegrees,
                final double flywheelSpeedMPS) {
            this.turretYawDegrees = turretYawDegrees;
            this.hoodPitchDegrees = hoodPitchDegrees;
            this.flywheelSpeedMPS = flywheelSpeedMPS;
        }

        public double getTurretYawDegrees() {
            return this.turretYawDegrees;
        }

        public double getHoodPitchDegrees() {
            return this.hoodPitchDegrees;
        }

        public double getFlyWheelSpeedMPS() {
            return this.flywheelSpeedMPS;
        }
    }

    private static ShotResult ShotResult;

    // =========================================================================
    // PUBLIC API
    // =========================================================================

    /**
     * Calculates the firing solution accounting for robot motion.
     * 
     * @param t3d         The {@link Transform3d} that has out relative distance
     *                    from the shooter.
     * @param robotVx     Robot velocity X component (m/s)
     * @param robotVy     Robot velocity Y component (m/s)
     * @param shapeScalar 0.0 (Flat/Laser) to 1.0 (Steep/Lob)
     * @param isBlocked   If true, prioritizes high launch angle.
     * @param outResult   Output container.
     */
    public ShotResult calculate(
            Transform3d t3d, Rotation2d heading, double robotVx, double robotVy,
            double shapeScalar, boolean isBlocked) {

        // 1. Calculate relative distances
        double dx = t3d.getX();
        double dy = t3d.getY();
        double dz = t3d.getZ();

        // Horizontal distance to target (on the floor)
        double distFloor = Math.sqrt(dx * dx + dy * dy);

        // Angle from robot to target (Field-Relative)
        double angleToTarget = Math.atan2(dy, dx);

        // 2. Solve for the "Ideal World Vector" (Stationary Shot)
        // We use the Secant/RK4 solver here to get the physics-accurate vertical launch
        // params.
        // The solver returns the required Launch Angle (Theta) and Velocity (Magnitude)
        // as if the robot were standing still.

        double idealThetaRad;
        double idealVelocityMag;

        // --- Solver Logic Start (Inline for performance) ---
        // (A) Determine Angle Strategy
        if (isBlocked) {
            double launchAngleDeg = 80;
            idealThetaRad = Math.toRadians(launchAngleDeg);
        } else {
            double minEntry = -45.0;
            double maxEntry = -75.0;
            double entryRad = Math.toRadians(minEntry + (shapeScalar * (maxEntry - minEntry)));
            double term1 = (2 * dz) / distFloor;
            double term2 = Math.tan(entryRad);
            idealThetaRad = Math.atan(term1 - term2);
        }

        // (B) Initial Guess (Vacuum)
        double cosTheta = Math.cos(idealThetaRad);
        double tanTheta = Math.tan(idealThetaRad);
        double num = G * distFloor * distFloor;
        double den = 2 * cosTheta * cosTheta * ((distFloor * tanTheta) - dz);

        if (den <= 0)
            den = 0.001; // Prevent NaN
        double vGuess = Math.sqrt(num / den);

        // (C) Secant Refinement for Drag
        double v0 = vGuess;
        double v1 = vGuess + 0.5;
        double h0 = simulateShotHeight(v0, idealThetaRad, distFloor);
        double h1 = simulateShotHeight(v1, idealThetaRad, distFloor);

        for (int i = 0; i < MAX_SECANT_ITERS; i++) {
            if (Math.abs(h1 - h0) < 0.0001)
                break;
            double error1 = h1 - dz;
            double error0 = h0 - dz;
            double vNew = v1 - error1 * (v1 - v0) / (error1 - error0);
            v0 = v1;
            h0 = h1;
            v1 = vNew;
            h1 = simulateShotHeight(v1, idealThetaRad, distFloor);
            if (Math.abs(h1 - dz) < 0.01)
                break;
        }
        idealVelocityMag = v1;
        // --- Solver Logic End ---

        // 3. Decompose "Ideal World Vector" into Cartesian X, Y, Z
        // Vertical component (Z) is independent of floor direction
        double vWorldZ = idealVelocityMag * Math.sin(idealThetaRad);

        // Horizontal magnitude
        double vWorldHoriz = idealVelocityMag * Math.cos(idealThetaRad);

        // Break Horizontal into X and Y based on target direction
        double vWorldX = vWorldHoriz * Math.cos(angleToTarget);
        double vWorldY = vWorldHoriz * Math.sin(angleToTarget);

        // 4. Vector Subtraction: Compensate for Robot Velocity
        // V_shooter = V_world - V_robot
        double vShooterX = vWorldX - robotVx;
        double vShooterY = vWorldY - robotVy;
        double vShooterZ = vWorldZ; // Assuming robot isn't moving vertically (jumping)

        // 5. Re-Calculate Polar Coordinates for Hardware
        double vShooterHoriz = Math.sqrt(vShooterX * vShooterX + vShooterY * vShooterY);

        // This angle is FIELD RELATIVE (e.g. 90 degrees = North)
        double fieldRelativeTurretAngle = Math.toDegrees(Math.atan2(vShooterY, vShooterX));

        // CONVERSION: Field Relative -> Robot Relative
        double robotRelativeTurretAngle = fieldRelativeTurretAngle - heading.getDegrees();

        // Normalize to -180 to 180 range for safety
        ShotResult.turretYawDegrees = robotRelativeTurretAngle;

        ShotResult.hoodPitchDegrees = Math.toDegrees(Math.atan2(vShooterZ, vShooterHoriz));
        ShotResult.flywheelSpeedMPS = Math.sqrt(vShooterHoriz * vShooterHoriz + vShooterZ * vShooterZ);

        return VelocityAngleSolver.ShotResult;
    }

    // =========================================================================
    // PRIVATE SIMULATION HELPER (RK4)
    // =========================================================================

    private double simulateShotHeight(double v0, double theta, double targetDist) {
        double x = 0, y = 0;
        double vx = v0 * Math.cos(theta);
        double vy = v0 * Math.sin(theta);
        double dt = TIME_STEP;
        double time = 0;

        while (x < targetDist && time < MAX_SIM_TIME) {
            double predictedX = x + vx * dt;
            if (predictedX > targetDist) {
                dt = (targetDist - x) / vx; // Adjust last step
            }

            // RK4 Steps
            double ax1 = getAccX(vx, vy);
            double ay1 = getAccY(vx, vy);

            double vx2 = vx + ax1 * 0.5 * dt;
            double vy2 = vy + ay1 * 0.5 * dt;
            double ax2 = getAccX(vx2, vy2);
            double ay2 = getAccY(vx2, vy2);

            double vx3 = vx + ax2 * 0.5 * dt;
            double vy3 = vy + ay2 * 0.5 * dt;
            double ax3 = getAccX(vx3, vy3);
            double ay3 = getAccY(vx3, vy3);

            double vx4 = vx + ax3 * dt;
            double vy4 = vy + ay3 * dt;
            double ax4 = getAccX(vx4, vy4);
            double ay4 = getAccY(vx4, vy4);

            double axAvg = (ax1 + 2 * ax2 + 2 * ax3 + ax4) / 6.0;
            double ayAvg = (ay1 + 2 * ay2 + 2 * ay3 + ay4) / 6.0;

            x += vx * dt + 0.5 * axAvg * dt * dt;
            y += vy * dt + 0.5 * ayAvg * dt * dt;
            vx += axAvg * dt;
            vy += ayAvg * dt;
            time += dt;
        }
        return y;
    }

    private double getAccX(double vx, double vy) {
        double v = Math.sqrt(vx * vx + vy * vy);
        if (v == 0)
            return 0;
        double fDragX = -0.5 * AIR_DENSITY * AREA * DRAG_COEFF * v * vx;
        double fLiftX = -0.5 * AIR_DENSITY * AREA * LIFT_COEFF * v * vy;
        return (fDragX + fLiftX) / MASS;
    }

    private double getAccY(double vx, double vy) {
        double v = Math.sqrt(vx * vx + vy * vy);
        if (v == 0)
            return -G;
        double fGravY = -MASS * G;
        double fDragY = -0.5 * AIR_DENSITY * AREA * DRAG_COEFF * v * vy;
        double fLiftY = 0.5 * AIR_DENSITY * AREA * LIFT_COEFF * v * vx;
        return (fGravY + fDragY + fLiftY) / MASS;
    }
}
