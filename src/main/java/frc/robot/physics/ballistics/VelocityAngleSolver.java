package frc.robot.physics.ballistics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Zero-Allocation Ballistic Solver for FRC RoboRIO 2 (WPILib 2026).
 *
 * <h3>Key Features:</h3>
 * <ul>
 * <li>Fixed hood (launchAngle is hardcoded — never changes at runtime)</li>
 * <li>Full drag + Magnus physics via RK4 integration</li>
 * <li>Secant solver to find the horizontal speed that hits the target</li>
 * <li>Moving Reference Frame: v_world = v_shooter + v_robot</li>
 * <li>Field-centric math (swerve + Pigeon 2 + CANivore)</li>
 * </ul>
 *
 * <h3>How the solver works (high-level):</h3>
 * 
 * <pre>
 * 1. Vision gives us a robot-frame Transform3d to the target.
 * 2. We extract horizontal distance d and height dz.
 * 3. We define "m" = the ball's world-frame horizontal speed along
 *    the line from shooter to target. If we pick the right m,
 *    the ball arrives at (d, dz) accounting for drag + Magnus.
 * 4. We guess m from the vacuum projectile formula, then refine
 *    with a secant root-finder over the RK4 simulation.
 * 5. The fixed-hood constraint tells us the flywheel speed:
 *      v_flywheel = |v_shooter_horiz| / cos(θ)
 *    where |v_shooter_horiz| accounts for robot velocity.
 * </pre>
 *
 * <h3>Usage in Shooter subsystem:</h3>
 * 
 * <pre>
 * ShotResult result = solver.calculate(targetT3d, heading, vxField, vyField, 80.0);
 * if (result.isValidShot()) {
 *     double rpm = rpmSolver.calculateMotorRPM(result.getFlywheelSpeedMPS());
 *     // ... send rpm to Kraken X60
 * }
 * </pre>
 */
public class VelocityAngleSolver {

    // =========================================================================
    // PHYSICAL CONSTANTS
    // =========================================================================

    /** Gravitational acceleration (m/s²). */
    private static final double G = 9.81;

    /** Mass of the game piece (kg). */
    private static final double MASS = 0.216;

    /** Radius of the game piece (m). */
    private static final double RADIUS = 0.075;

    /**
     * Cross-sectional area of the game piece (m²).
     * 
     * <pre>
     * A = π · r² ≈ 0.01767 m²
     * </pre>
     */
    private static final double AREA = Math.PI * RADIUS * RADIUS;

    /**
     * Air density at standard indoor conditions (kg/m³).
     * (20 °C, 101.325 kPa, 50 %RH → ρ ≈ 1.199)
     */
    private static final double AIR_DENSITY = 1.199;

    /**
     * Drag coefficient for a sphere at typical game-piece Reynolds numbers.
     * 
     * <pre>
     * C_d ≈ 0.47 for smooth sphere, Re ~ 10⁴–10⁵
     * </pre>
     */
    private static final double DRAG_COEFF = 0.47;

    /**
     * Magnus (lift) coefficient.
     * <p>
     * Encodes spin-dependent lift. Positive value assumes <b>backspin</b>
     * (top of ball rotates backward relative to motion → upward lift).
     * <p>
     * <b>If your flywheel imparts topspin instead, negate this constant.</b>
     * 
     * <pre>
     * F_lift = ½ · ρ · A · C_L · |v| · (ω̂ × v⃗)
     * </pre>
     */
    private static final double LIFT_COEFF = 0.031;

    // =========================================================================
    // SOLVER SETTINGS
    // =========================================================================

    /** Maximum iterations for the secant root-finder. */
    private static final int MAX_SECANT_ITERS = 5;

    /** RK4 integration time step (seconds). */
    private static final double TIME_STEP = 0.01;

    /** Maximum simulation duration — safety bailout (seconds). */
    private static final double MAX_SIM_TIME = 5.0;

    /**
     * Absolute ceiling on flywheel exit speed (m/s).
     * Also used as the upper clamp for the secant solver's m output.
     */
    private static final double MAX_VALID_SPEED_MPS = 35.0;

    /**
     * Maximum acceptable height error at the target (m).
     * If |simulated_height − target_height| exceeds this, shot is invalid.
     */
    private static final double MAX_HEIGHT_ERROR_M = 0.02;

    // =========================================================================
    // RESULT CONTAINER
    // =========================================================================

    /**
     * Immutable result. Zero allocation after construction.
     * Hood pitch is always the input launchAngle (fixed hardware).
     */
    public static class ShotResult {
        private final double turretYawDegrees;
        private final double hoodPitchDegrees;
        private final double flywheelSpeedMPS;
        private final boolean isValidShot;
        private final double simulationErrorMeters;

        private ShotResult(double turretYawDegrees,
                double hoodPitchDegrees,
                double flywheelSpeedMPS,
                boolean isValidShot,
                double simulationErrorMeters) {
            this.turretYawDegrees = turretYawDegrees;
            this.hoodPitchDegrees = hoodPitchDegrees;
            this.flywheelSpeedMPS = flywheelSpeedMPS;
            this.isValidShot = isValidShot;
            this.simulationErrorMeters = simulationErrorMeters;
        }

        /** Robot-relative yaw for turret PID (degrees). */
        public double getTurretYawDegrees() {
            return turretYawDegrees;
        }

        /** Always equals input launch angle — for logging/telemetry. */
        public double getHoodPitchDegrees() {
            return hoodPitchDegrees;
        }

        /** Ball exit speed (m/s). Feed to VelocityToRPMSolver. */
        public double getFlywheelSpeedMPS() {
            return flywheelSpeedMPS;
        }

        /** Stage-1 validity: math + geometry + drag convergence. */
        public boolean isValidShot() {
            return isValidShot;
        }

        /** Height error at target distance (m). For SmartDashboard. */
        public double getSimulationErrorMeters() {
            return simulationErrorMeters;
        }

        @Override
        public String toString() {
            return new StringBuilder().append("ShotResult{\n")
                    .append("\tturretYawDegrees = ").append(turretYawDegrees)
                    .append("\n\thoodPitchDegrees = ").append(hoodPitchDegrees)
                    .append("\n\tflywheelSpeedMPS = ").append(flywheelSpeedMPS)
                    .append("\n\tisValidShot = ").append(isValidShot)
                    .append("\n\tsimulationErrorMeters = ").append(simulationErrorMeters)
                    .append("\n}").toString();
        }
    }

    // =========================================================================
    // PUBLIC API
    // =========================================================================

    /**
     * Calculates a full firing solution with robot-motion compensation.
     *
     * <h3>Moving reference frame math:</h3>
     * 
     * <pre>
     *   v⃗_ball_world  = v⃗_ball_shooter + v⃗_robot
     *   v⃗_ball_shooter = v⃗_ball_world  − v⃗_robot
     *
     *   Fixed hood constraint (angle θ from horizontal):
     *     v_vertical       = |v⃗_shooter_horiz| · tan(θ)
     *     v_flywheel_total = |v⃗_shooter_horiz| / cos(θ)
     * </pre>
     *
     * @param t3d         Transform3d from robot to target (robot-frame)
     * @param heading     Robot heading in field frame (Pigeon 2)
     * @param robotVx     Field-centric chassis velocity X (m/s)
     * @param robotVy     Field-centric chassis velocity Y (m/s)
     * @param launchAngle Fixed hood angle from horizontal (degrees), e.g. 80.0
     * @return ShotResult with turret yaw, flywheel m/s, and validity flag
     */
    public ShotResult calculate(
            Transform3d t3d,
            Rotation2d heading,
            double robotVx,
            double robotVy,
            double launchAngle) {

        // ─────────────────────────────────────────────────────────────
        // 1. Extract target displacement (robot-frame)
        // ─────────────────────────────────────────────────────────────
        double dx = t3d.getX(); // forward (robot frame)
        double dy = t3d.getY(); // left (robot frame)
        double dz = t3d.getZ(); // up (robot frame ≡ field frame for Z)

        // Horizontal distance — magnitude is frame-invariant under yaw rotation.
        double distFloor = Math.hypot(dx, dy);

        // Bail out for degenerate / NaN cases
        if (distFloor < 0.1 || Double.isNaN(distFloor)) {
            return new ShotResult(0.0, launchAngle, 0.0, false, 999.0);
        }

        // ─────────────────────────────────────────────────────────────
        // 2. Convert bearing angle to field-frame
        //
        // φ_robot = atan2(dy, dx) (bearing in robot frame)
        // φ_field = φ_robot + heading (bearing in field frame)
        //
        // We need field-frame because robotVx, robotVy are field-frame.
        // ─────────────────────────────────────────────────────────────
        double robotPhiRad = Math.atan2(dy, dx);
        double fieldPhiRad = robotPhiRad + heading.getRadians();
        double phi = fieldPhiRad;
        double cosPhi = Math.cos(phi);
        double sinPhi = Math.sin(phi);

        // ─────────────────────────────────────────────────────────────
        // 3. Fixed launch-angle trig
        //
        // θ = launch angle from horizontal
        // cos(θ) → projects total speed onto horizontal
        // tan(θ) → converts horizontal speed to vertical
        // ─────────────────────────────────────────────────────────────
        double thetaRad = Math.toRadians(launchAngle);
        double cosTheta = Math.cos(thetaRad);
        double tanTheta = Math.tan(thetaRad);

        // ─────────────────────────────────────────────────────────────
        // 4. Vacuum initial guess for m (horizontal speed)
        //
        // From projectile kinematics (no drag, no Magnus):
        //
        // h = d·tan(θ) − g·d² / [2·v²·cos²(θ)]
        //
        // Solving for total speed v when h = dz:
        //
        // v = √[ g·d² / (2·cos²(θ)·(d·tan(θ) − dz)) ]
        //
        // Horizontal component: m = v·cos(θ)
        //
        // This UNDERESTIMATES the true required speed because drag
        // slows the ball. The secant solver corrects upward from here.
        // ─────────────────────────────────────────────────────────────
        double num = G * distFloor * distFloor;
        double den = 2.0 * cosTheta * cosTheta * (distFloor * tanTheta - dz);
        if (den <= 0.0) {
            // Impossible geometry at this angle — target is above the
            // maximum vacuum arc. Force a large guess so the solver
            // can try, and let the validity check reject if needed.
            den = 0.001;
        }
        double vWorldGuess = Math.sqrt(num / den);
        double mGuess = vWorldGuess * cosTheta;

        // ─────────────────────────────────────────────────────────────
        // 5. Secant root-finder
        //
        // We seek m such that:
        // f(m) = simulatedHeight(m) − dz = 0
        //
        // Secant update rule:
        // m_new = m₁ − f(m₁) · (m₁ − m₀) / [f(m₁) − f(m₀)]
        //
        // Convergence order ≈ 1.618 (golden ratio). From a
        // reasonable start, 5 iterations ≈ 10+ correct digits.
        // ─────────────────────────────────────────────────────────────
        double m0 = mGuess;
        double m1 = mGuess + 0.5;

        double h0 = computeHeightForM(m0, robotVx, robotVy, cosPhi, sinPhi, tanTheta, distFloor);
        double h1 = computeHeightForM(m1, robotVx, robotVy, cosPhi, sinPhi, tanTheta, distFloor);

        boolean converged = false;
        for (int i = 0; i < MAX_SECANT_ITERS; i++) {

            // ── [FIX #2] Flat-function guard ──
            //
            // If h(m₀) ≈ h(m₁), the secant slope is near-zero and the
            // next step would divide by ~0 → huge/NaN jump.
            //
            // OLD CODE set `converged = true` here unconditionally.
            // That's wrong: a flat region FAR from dz is NOT a solution.
            //
            // NEW: only declare convergence if we're also within tolerance.
            if (Math.abs(h1 - h0) < 0.0001) {
                converged = Math.abs(h1 - dz) < MAX_HEIGHT_ERROR_M;
                break;
            }

            double error1 = h1 - dz;
            double error0 = h0 - dz;
            double mNew = m1 - error1 * (m1 - m0) / (error1 - error0);

            // ── [FIX #3] Clamp m to a physically meaningful range ──
            //
            // Without this, the secant method can produce:
            // • m < 0 → ball travels backward, RK4 loops for
            // MAX_SIM_TIME (5 s!) burning all your CPU
            // • m → ∞ → unreachable flywheel speed, wastes iterations
            //
            // Lower bound 0.1 m/s: at distFloor ≥ 0.1 m and 80° launch,
            // the vacuum guess is always > 1 m/s, so 0.1 is safe.
            // Upper bound MAX_VALID_SPEED_MPS: any m beyond 35 produces
            // a flywheel speed well over 35/cos(θ) which fails validity.
            mNew = Math.max(0.1, Math.min(mNew, MAX_VALID_SPEED_MPS));

            m0 = m1;
            h0 = h1;
            m1 = mNew;
            h1 = computeHeightForM(m1, robotVx, robotVy, cosPhi, sinPhi, tanTheta, distFloor);

            if (Math.abs(h1 - dz) < 0.01) {
                converged = true;
                break;
            }
        }

        // ─────────────────────────────────────────────────────────────
        // 6. Derive flywheel speed from the converged m
        //
        // World-frame ball velocity (horizontal, along target line):
        // v_world = (m·cosφ, m·sinφ)
        //
        // Shooter-relative horizontal velocity:
        // v_shooter = v_world − v_robot
        // |v_sh| = |(m·cosφ − Vx_r, m·sinφ − Vy_r)|
        //
        // Fixed-hood constraint:
        // flywheel_speed = |v_sh| / cos(θ)
        //
        // This is the TOTAL ball exit speed in the shooter frame.
        // ─────────────────────────────────────────────────────────────
        double finalM = m1;
        double vShooterX = finalM * cosPhi - robotVx;
        double vShooterY = finalM * sinPhi - robotVy;
        double vHorizRelMag = Math.hypot(vShooterX, vShooterY);

        double flywheelSpeedMPS = (cosTheta > 0.001)
                ? vHorizRelMag / cosTheta
                : 0.0;

        // ─────────────────────────────────────────────────────────────
        // 7. Turret yaw (robot-relative)
        //
        // With shot-leading disabled, we aim directly at the target.
        // φ is field-frame bearing; subtract heading → robot-frame.
        //
        // [FIX #5] Use MathUtil.inputModulus for correct wrapping.
        //
        // Java's % operator preserves the sign of the DIVIDEND:
        // (-200 + 180) % 360 = (-20) % 360 = -20 → -20 - 180 = -200 WRONG
        //
        // MathUtil.inputModulus uses floored division, so:
        // inputModulus(-200, -180, 180) = 160 ✓
        //
        // [FIX #1] Removed System.out.println calls that were here.
        // String allocation + GC pauses on RoboRIO = jittery control.
        // ─────────────────────────────────────────────────────────────
        double fieldYawDeg = Math.toDegrees(phi);
        double robotRelativeYaw = fieldYawDeg - heading.getDegrees();
        robotRelativeYaw = MathUtil.inputModulus(robotRelativeYaw, -180.0, 180.0);

        // ─────────────────────────────────────────────────────────────
        // 8. Validity — Stage 1 (math / geometry / convergence)
        //
        // Stage 2 (hardware RPM limit) is checked in the command layer
        // after converting m/s → RPM via VelocityToRPMSolver.
        // ─────────────────────────────────────────────────────────────
        double simError = Math.abs(h1 - dz);
        boolean valid = converged
                && simError <= MAX_HEIGHT_ERROR_M
                && flywheelSpeedMPS > 0.0
                && flywheelSpeedMPS <= MAX_VALID_SPEED_MPS;

        return new ShotResult(
                robotRelativeYaw,
                launchAngle,
                flywheelSpeedMPS,
                valid,
                simError);
    }

    // =========================================================================
    // PRIVATE HELPERS
    // =========================================================================

    /**
     * Evaluates the simulated arrival height for a given world horizontal
     * speed m. Called once per secant iteration.
     *
     * <h3>Fixed-hood constraint (derives vZ from m):</h3>
     * 
     * <pre>
     *   v⃗_shooter_horiz = (m·cosφ − Vx_robot,  m·sinφ − Vy_robot)
     *   |v_sh|           = hypot(above)
     *   vZ               = |v_sh| · tan(θ)
     * </pre>
     *
     * The vertical velocity equals the <i>shooter-relative</i> horizontal
     * speed times tan(θ), because the hood forces a fixed exit angle.
     * Since the robot only moves horizontally, vZ is identical in both
     * the shooter frame and the world frame.
     *
     * @param m          World-frame horizontal speed along target line (m/s)
     * @param robotVx    Field-frame robot velocity X (m/s)
     * @param robotVy    Field-frame robot velocity Y (m/s)
     * @param cosPhi     cos(field bearing to target)
     * @param sinPhi     sin(field bearing to target)
     * @param tanTheta   tan(launch angle)
     * @param targetDist Horizontal distance to target (m)
     * @return Simulated height (m) at targetDist
     */
    private double computeHeightForM(double m,
            double robotVx, double robotVy,
            double cosPhi, double sinPhi,
            double tanTheta, double targetDist) {
        double vShooterX = m * cosPhi - robotVx;
        double vShooterY = m * sinPhi - robotVy;
        double vHorizRelMag = Math.hypot(vShooterX, vShooterY);
        double vZ = vHorizRelMag * tanTheta;

        return simulateShotHeight(m, vZ, targetDist);
    }

    /**
     * Full RK4 trajectory simulation with drag + Magnus forces.
     *
     * <h3>Coordinate system (2D vertical plane along target line):</h3>
     * 
     * <pre>
     *   x  = horizontal distance toward target (m)
     *   y  = vertical height above launch point (m)
     *   vx = horizontal velocity  (positive → toward target)
     *   vy = vertical velocity    (positive → upward)
     * </pre>
     *
     * <h3>Why 2D is exact here (not an approximation):</h3>
     * <p>
     * By construction, the ball's world-frame horizontal velocity is
     * <b>entirely</b> along the target line (magnitude m, direction φ).
     * There is no lateral component in the world frame, so the full 3D
     * trajectory lies in this vertical plane, and the 2D speed used for
     * drag equals the true 3D speed. No information is lost.
     *
     * @param vx0        World horizontal speed along target line (m/s)
     * @param vy0        World vertical speed (m/s, positive = up)
     * @param targetDist Horizontal distance to target (m)
     * @return Height (m) when ball reaches targetDist horizontally
     */
    private double simulateShotHeight(double vx0, double vy0, double targetDist) {
        double x = 0.0;
        double y = 0.0;
        double vx = vx0;
        double vy = vy0;
        double time = 0.0;

        while (x < targetDist && time < MAX_SIM_TIME) {
            // ── [FIX #4a] Reset dt every iteration ──
            //
            // The original code set dt = TIME_STEP once before the loop.
            // If the final-step shortening below fires but x still doesn't
            // quite reach targetDist (floating-point), the next iteration
            // would reuse the tiny dt. Resetting avoids that edge case.
            double dt = TIME_STEP;

            // ── [FIX #4b] Guard: ball stalled or reversed ──
            //
            // If drag (+ Magnus on ascent) bleeds vx to zero, the ball
            // can never reach targetDist. Without this guard:
            // • dt = (targetDist − x) / vx → divide-by-zero or negative dt
            // • The while-loop runs until MAX_SIM_TIME (5 s of CPU burn)
            if (vx <= 0.0) {
                break;
            }

            // Shorten final step to land exactly on targetDist.
            // dt_final = remaining_distance / current_speed
            double predictedX = x + vx * dt;
            if (predictedX > targetDist) {
                dt = (targetDist - x) / vx;
            }

            // ─────────────────────────────────────────────────────────
            // RK4 integration of the coupled ODE system:
            //
            // dx/dt = vx dy/dt = vy
            // dvx/dt = ax(vx, vy) dvy/dt = ay(vx, vy)
            //
            // Four stages evaluate the acceleration at different points
            // within the time step, then combine with weighted average.
            // ─────────────────────────────────────────────────────────

            // Stage 1: slopes at current state (t_n)
            double ax1 = getAccX(vx, vy);
            double ay1 = getAccY(vx, vy);

            // Stage 2: slopes at midpoint using Stage 1 derivatives
            double vx2 = vx + ax1 * 0.5 * dt;
            double vy2 = vy + ay1 * 0.5 * dt;
            double ax2 = getAccX(vx2, vy2);
            double ay2 = getAccY(vx2, vy2);

            // Stage 3: slopes at midpoint using Stage 2 derivatives
            double vx3 = vx + ax2 * 0.5 * dt;
            double vy3 = vy + ay2 * 0.5 * dt;
            double ax3 = getAccX(vx3, vy3);
            double ay3 = getAccY(vx3, vy3);

            // Stage 4: slopes at endpoint using Stage 3 derivatives
            double vx4 = vx + ax3 * dt;
            double vy4 = vy + ay3 * dt;
            double ax4 = getAccX(vx4, vy4);
            double ay4 = getAccY(vx4, vy4);

            // ── [FIX #7] Proper RK4 position update ──
            //
            // For the position ODE dx/dt = vx(t), the four RK4 stages
            // evaluate vx at different times within the step:
            //
            // k₁ˣ = vx · Δt
            // k₂ˣ = (vx + ½a₁Δt) · Δt
            // k₃ˣ = (vx + ½a₂Δt) · Δt
            // k₄ˣ = (vx + a₃Δt) · Δt
            //
            // Δx = (k₁ + 2k₂ + 2k₃ + k₄) / 6
            // = vx·Δt + [(a₁ + a₂ + a₃) / 6] · Δt²
            //
            // KEY INSIGHT: the position weighting on accelerations is
            // (1, 1, 1, 0) / 6
            // while the velocity weighting is the familiar
            // (1, 2, 2, 1) / 6
            //
            // The old code used the velocity weights (1,2,2,1)/6 for
            // position too (via 0.5 * axAvg), introducing an error of
            // (a₁ − a₄)/12 · Δt²
            // per step. Small (~mm), but wrong on principle.
            x += vx * dt + (ax1 + ax2 + ax3) / 6.0 * dt * dt;
            y += vy * dt + (ay1 + ay2 + ay3) / 6.0 * dt * dt;

            // Velocity update: standard RK4 weighted average
            // Δvx = [(a₁ + 2a₂ + 2a₃ + a₄) / 6] · Δt
            vx += (ax1 + 2.0 * ax2 + 2.0 * ax3 + ax4) / 6.0 * dt;
            vy += (ay1 + 2.0 * ay2 + 2.0 * ay3 + ay4) / 6.0 * dt;

            time += dt;
        }
        return y;
    }

    // =========================================================================
    // AERODYNAMIC FORCE MODEL
    // =========================================================================

    /**
     * Horizontal acceleration from drag + Magnus forces.
     *
     * <h3>Drag force (opposes velocity):</h3>
     * 
     * <pre>
     *   F⃗_drag = −½ ρ C_d A |v⃗| · v⃗
     *
     *   a_drag,x = −(½ ρ C_d A) · |v| · vx / m_ball
     * </pre>
     *
     * <h3>Magnus force (perpendicular to velocity):</h3>
     * 
     * <pre>
     *   F⃗_Magnus = ½ ρ C_L A |v⃗| · (ω̂ × v⃗)
     *
     *   For backspin with ω̂ = +ẑ (out of the 2D plane):
     *     ẑ × (vx, vy) = (−vy, +vx)
     *
     *   a_Magnus,x = −(½ ρ C_L A) · |v| · vy / m_ball
     * </pre>
     *
     * Physical interpretation: when the ball moves forward and upward,
     * Magnus pushes it backward (−x). On the downward arc, Magnus
     * pushes it forward (+x). These partially cancel over a full flight.
     *
     * @param vx Horizontal velocity (m/s)
     * @param vy Vertical velocity (m/s)
     * @return Horizontal acceleration (m/s²)
     */
    private double getAccX(double vx, double vy) {
        double v = Math.hypot(vx, vy);
        if (v == 0.0)
            return 0.0;
        double fDragX = -0.5 * AIR_DENSITY * AREA * DRAG_COEFF * v * vx;
        double fLiftX = -0.5 * AIR_DENSITY * AREA * LIFT_COEFF * v * vy;
        return (fDragX + fLiftX) / MASS;
    }

    /**
     * Vertical acceleration from gravity + drag + Magnus.
     *
     * <pre>
     *   a_y = −g
     *       + [−½ ρ C_d A |v| · vy] / m_ball       (drag, opposes vy)
     *       + [+½ ρ C_L A |v| · vx] / m_ball       (Magnus lift)
     * </pre>
     *
     * The Magnus term <code>+C_L · |v| · vx</code> is the classic backspin
     * lift: when the ball moves forward (vx &gt; 0), spin creates an upward
     * force that extends range beyond what a drag-only model predicts.
     *
     * @param vx Horizontal velocity (m/s)
     * @param vy Vertical velocity (m/s)
     * @return Vertical acceleration (m/s²)
     */
    private double getAccY(double vx, double vy) {
        double v = Math.hypot(vx, vy);
        if (v == 0.0)
            return -G;
        double fGravY = -MASS * G;
        double fDragY = -0.5 * AIR_DENSITY * AREA * DRAG_COEFF * v * vy;
        double fLiftY = 0.5 * AIR_DENSITY * AREA * LIFT_COEFF * v * vx;
        return (fGravY + fDragY + fLiftY) / MASS;
    }
}