package frc.robot.commands.shoot;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.physics.ballistics.VelocityAngleSolver;
import frc.robot.physics.ballistics.VelocityAngleSolver.ShotResult;
import frc.robot.physics.rotational.VelocityToRPMSolver;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.AimCamera;

public class LockOnShootAndDrive extends Command {
        // =================================================================
        // Tuning Constants
        // =================================================================
        private static final float LAUNCH_ANGLE_DEGREES = 80;
        private static final float TOO_CLOSE_INCHES = 25;
        private static final float TOO_FAR_INCHES = 150;

        /**
         * EMA smoothing coefficient for the heading setpoint.
         *
         * <pre>
         * θ_smooth[n] = θ_smooth[n-1] + α · wrap(θ_raw[n] − θ_smooth[n-1])
         * </pre>
         *
         * At 50 Hz (Δt = 0.02 s), time-constant τ = Δt / α :
         * <ul>
         * <li>α = 0.05 → τ = 0.40 s (very smooth, 95% in ~1.2 s)</li>
         * <li>α = 0.10 → τ = 0.20 s (balanced, 95% in ~0.6 s) ← START HERE</li>
         * <li>α = 0.15 → τ = 0.13 s (responsive, 95% in ~0.4 s)</li>
         * </ul>
         *
         * ±2° vision noise × 0.10 = ±0.2° after filter.
         * With P = 10, that produces only 0.035 rad/s of jitter — invisible.
         */
        private static final double HEADING_ALPHA = 0.15;

        /** Creep speed toward / away from hub (m/s). ≈ 0.5 ft/s */
        private static final double CREEP_MPS = 0.15;

        // =================================================================
        // Subsystems
        // =================================================================
        private final Shooter shooter;
        private final Feeder feeder;
        private final CommandSwerveDrivetrain drive;

        // =================================================================
        // Vision
        // =================================================================
        private final AimCamera aimCamera;

        // =================================================================
        // Driver Inputs (joystick X → strafe, joystick Y → fwd/back)
        // =================================================================
        private final DoubleSupplier xSupplier;
        private final DoubleSupplier ySupplier;

        // =================================================================
        // Solvers
        // =================================================================
        private final VelocityAngleSolver vaSolver;
        private final VelocityToRPMSolver vRpmSolver;

        // =================================================================
        // Swerve request (reused every frame — zero allocation)
        // =================================================================
        private final SwerveRequest.FieldCentricFacingAngle facingAngle;

        // =================================================================
        // Filter state (reset on every command start)
        // =================================================================

        /** EMA-filtered heading setpoint (radians, field-relative). */
        private double smoothedHeadingRad;

        /**
         * Last heading we computed from a real vision frame.
         * Survives dropout frames so the robot holds aim instead
         * of snapping or stopping.
         */
        private Rotation2d lastValidTarget;

        // =================================================================
        // Constructor
        // =================================================================
        public LockOnShootAndDrive(
                        final Shooter shooter,
                        final CommandSwerveDrivetrain drive,
                        final Feeder feeder,
                        final AimCamera aimCamera,
                        final DoubleSupplier xMove,
                        final DoubleSupplier yMove,
                        final double maxSpeed) {

                this.shooter = shooter;
                this.feeder = feeder;
                this.drive = drive;
                this.aimCamera = aimCamera;
                this.xSupplier = xMove;
                this.ySupplier = yMove;

                this.vaSolver = new VelocityAngleSolver();
                this.vRpmSolver = new VelocityToRPMSolver(() -> shooter.getMotorRPM());

                this.facingAngle = new SwerveRequest.FieldCentricFacingAngle()
                                .withDeadband(maxSpeed * 0.1)
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                                //
                                // FIX 4 (was 6.0 rad/s):
                                // 6 rad/s deadband meant the heading PID had to
                                // output >6 rad/s before ANY rotation happened.
                                // With P=20 that required >17° of error, so the
                                // robot sat still then SNAPPED — the exact symptom
                                // you described.
                                //
                                // 0.1 lets through everything except the tiniest
                                // residual noise the EMA didn't fully kill.
                                //
                                .withRotationalDeadband(0.1)
                                //
                                // FIX 5 (was P=20, I=0, D=0.3):
                                // P 10 — half the noise amplification, still fast.
                                // I 0 — no integral wind-up.
                                // D 0.8 — damping so heading settles without ringing.
                                //
                                // With the EMA removing ~90% of noise, P=10 gives:
                                // ω_jitter = 10 × 0.10 × 0.035 ≈ 0.035 rad/s
                                // That's 2°/s — completely invisible.
                                //
                                .withHeadingPID(8, 0, 0.5);

                addRequirements(this.shooter, this.drive);
        }

        // =================================================================
        // Lifecycle
        // =================================================================

        @Override
        public void initialize() {
                // Seed the filter with the current heading so there
                // is zero initial error — no snap on command start.
                smoothedHeadingRad = drive.getState().Pose.getRotation().getRadians();
                lastValidTarget = null;
        }

        @Override
        public void execute() {
                /*
                 * =============================================================
                 * 1. Snapshot current state
                 * =============================================================
                 */
                final Rotation2d heading = drive.getState().Pose.getRotation();
                final ChassisSpeeds fieldV = ChassisSpeeds.fromRobotRelativeSpeeds(
                                drive.getState().Speeds, heading);

                // Start with driver sticks (joystick Y → field X, X → field Y)
                double vx = ySupplier.getAsDouble();
                double vy = xSupplier.getAsDouble();

                // Will be set below — either from vision or from cache.
                Rotation2d rawTarget;

                /*
                 * =============================================================
                 * 2. Vision + ballistic solver
                 *
                 * FIX 2 & 3: ONE code path. No early returns.
                 * Every frame flows all the way to the single
                 * drive.setControl() at the bottom.
                 * =============================================================
                 */
                final Transform3d hub = aimCamera.getHubRelativeLocation();

                if (hub != null) {
                        /* ---- fresh vision frame ---- */
                        final ShotResult shot = vaSolver.calculate(
                                        hub, heading,
                                        fieldV.vxMetersPerSecond,
                                        fieldV.vyMetersPerSecond,
                                        LAUNCH_ANGLE_DEGREES);

                        // Raw heading (noisy — the EMA in step 3 cleans it)
                        rawTarget = heading.plus(
                                        Rotation2d.fromDegrees(shot.getTurretYawDegrees()));

                        // FIX 3: cache so dropout frames keep aim
                        lastValidTarget = rawTarget;

                        // ---- distance nudge (override fwd/back only) ----
                        final double dist = hub.getMeasureX().in(Inches);
                        if (dist > TOO_FAR_INCHES) {
                                vx = CREEP_MPS;
                        } else if (dist < TOO_CLOSE_INCHES) {
                                vx = -CREEP_MPS;
                        }

                        // ---- shooter + feeder ----
                        if (shot.isValidShot()) {
                                final double rpm = vRpmSolver.calculateMotorRPM(
                                                shot.getFlywheelSpeedMPS());

                                if (rpm <= shooter.getMaxRPM()) {
                                        shooter.setRPM(rpm);
                                        if (vRpmSolver.isReadyToFire()) {
                                                feeder.setFeederSpeed(0.6);
                                        } else {
                                                feeder.setFeederSpeed(0.0);
                                        }

                                } else {
                                        // RPM ceiling hit — get closer
                                        vx = CREEP_MPS;
                                }
                        }
                        //
                        // Invalid shot? We still AIM — just don't fire.
                        // This fixes: "invalid shots cancel my command".
                        // No fault, no cancel. Wait for conditions to improve.
                        //
                } else {
                        /*
                         * ---- camera dropout frame ----
                         *
                         * FIX 3: At 30 fps camera / 50 Hz robot, ~40% of
                         * frames have no vision. Instead of stopping the
                         * robot (old code), we hold the last known heading
                         * and let the driver keep translating.
                         *
                         * The EMA in step 3 just keeps outputting the last
                         * smoothed value — heading holds rock-steady.
                         */
                        rawTarget = (lastValidTarget != null)
                                        ? lastValidTarget
                                        : heading; // never seen a target yet — face forward
                }

                /*
                 * =============================================================
                 * 3. EMA low-pass filter on heading
                 *
                 * FIX 1: This replaces the SlewRateLimiter.
                 *
                 * The SlewRateLimiter limited the rate of change of
                 * the ERROR signal (which oscillates around zero from
                 * noise). That dampened the oscillation amplitude
                 * but never eliminated it.
                 *
                 * The EMA works on the SETPOINT itself:
                 * θ_smooth += α · wrap(θ_raw − θ_smooth)
                 *
                 * Each frame, only α (10%) of the raw jump is let
                 * through. High-frequency noise (alternating ±2°)
                 * cancels itself out over a few frames. Sustained
                 * real changes (robot drives, target moves) are
                 * tracked within ~0.6 s (95%).
                 *
                 * angleModulus handles the ±π wraparound correctly.
                 * =============================================================
                 */
                final double headingErr = MathUtil.angleModulus(
                                rawTarget.getRadians() - smoothedHeadingRad);
                smoothedHeadingRad = MathUtil.angleModulus(
                                smoothedHeadingRad + HEADING_ALPHA * headingErr);

                /*
                 * =============================================================
                 * 4. ONE drive command — every frame, no exceptions.
                 *
                 * FIX 2: The old code had up to 5 different
                 * setControl() calls with early returns that
                 * bypassed the smoothing. Now there is exactly
                 * one, and it always uses the filtered heading.
                 * =============================================================
                 */
                drive.setControl(facingAngle
                                .withVelocityX(vx)
                                .withVelocityY(vy)
                                .withTargetDirection(
                                                Rotation2d.fromRadians(smoothedHeadingRad)));
        }

        @Override
        public boolean isFinished() {
                // Runs until the driver releases the button (interrupted).
                // Invalid shots no longer cancel the command.
                return false;
        }

        @Override
        public void end(boolean interrupted) {
                feeder.setFeederSpeed(0);
                shooter.stopShooter();
        }
}