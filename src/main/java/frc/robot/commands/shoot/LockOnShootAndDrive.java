package frc.robot.commands.shoot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.AimCamera;

import frc.robot.physics.ballistics.VelocityAngleSolver;
import frc.robot.physics.ballistics.VelocityAngleSolver.ShotResult;
import frc.robot.physics.rotational.VelocityToRPMSolver;

public class LockOnShootAndDrive extends Command {
        // =============================================================================================================
        // Constants
        // =============================================================================================================
        private final float LAUNCH_ANGLE_DEGREES = 80;
        private final float TOO_CLOSE_DISTANCE_INCHES = 25;
        private final float TOO_FAR_DISTANCE_INCHES = 150;

        // =============================================================================================================
        // Sub-Systems
        // =============================================================================================================
        private final Shooter shooter;
        private final Feeder feeder;
        private final CommandSwerveDrivetrain drive;

        // =============================================================================================================
        // Systems
        // =============================================================================================================
        private final AimCamera aimCamera;

        // =============================================================================================================
        // Data Suppliers
        // =============================================================================================================
        // Allow driver to still move X/Y
        private final DoubleSupplier xSupplier;
        private final DoubleSupplier ySupplier;

        // =============================================================================================================
        // Solvers
        // =============================================================================================================
        private final VelocityAngleSolver vaSolver;
        private final VelocityToRPMSolver vRpmSolver;

        // =============================================================================================================
        // Swerve Drive Configurations
        // =============================================================================================================
        private final SwerveRequest.FieldCentricFacingAngle fieldFacingAngle;

        private boolean fault = false;

        // =============================================================================================================
        // Public Methods
        // =============================================================================================================
        public LockOnShootAndDrive(final Shooter shooter, final CommandSwerveDrivetrain drive, final Feeder feeder,
                        final AimCamera aimCamera, final DoubleSupplier xMove, final DoubleSupplier yMove,
                        final DoubleSupplier batteryVoltageSupplier, final double MaxSpeed) {
                this.shooter = shooter;
                this.feeder = feeder;
                this.drive = drive;
                this.aimCamera = aimCamera;
                this.xSupplier = xMove;
                this.ySupplier = yMove;

                this.vaSolver = new VelocityAngleSolver();
                this.vRpmSolver = new VelocityToRPMSolver(batteryVoltageSupplier, () -> shooter.getMotorRPM());
                this.fieldFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
                                // Add a 10% deadband
                                .withDeadband(MaxSpeed * 0.1)
                                // Use open-loop control for drive motors
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                                .withRotationalDeadband(6)
                                .withHeadingPID(20, 0, 0.3);

                // REQUIRE BOTH: This stops any other drive or shooter commands
                addRequirements(this.shooter, this.drive);
        }

        // -------------------------------------------------------------------------------------------------------------
        @Override
        public void execute() {
                // System.out.println("Executing lock on.");
                /*
                 * 1. One physics calculation to rule them all
                 */
                // Get required speed data.
                final ChassisSpeeds robotCentric = drive.getState().Speeds;
                final Rotation2d heading = drive.getState().Pose.getRotation();
                final ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(robotCentric, heading);
                final double fieldVx = fieldRelative.vxMetersPerSecond; // x in field frame (blue origin forward)
                final double fieldVy = fieldRelative.vyMetersPerSecond; // y in field frame

                // Get target data.
                final Transform3d hubRelativeTransform = aimCamera.getHubRelativeLocation();
                if (hubRelativeTransform == null) {
                        /*
                         * NOTE: Not keeping up with frames; or too little frames.
                         */
                        // System.err.println("No target");
                        drive.setControl(fieldFacingAngle
                                        .withVelocityX(0)
                                        .withVelocityY(0));
                        // fault = true;
                        // this.cancel();
                        return;
                }

                // Do some physics.
                final ShotResult shot = vaSolver.calculate(hubRelativeTransform, heading, fieldVx, fieldVy,
                                LAUNCH_ANGLE_DEGREES);
                // System.out.println(shot.toString());
                final Rotation2d targetHeading = heading.plus(Rotation2d.fromDegrees(shot.getTurretYawDegrees()));
                // System.out.println("targetHeading = " + heading.getDegrees());

                if (hubRelativeTransform.getMeasureX().in(Inches) >= TOO_FAR_DISTANCE_INCHES) {
                        drive.setControl(fieldFacingAngle
                                        .withVelocityX(LinearVelocity.ofRelativeUnits(0.1, FeetPerSecond))
                                        .withVelocityY(xSupplier.getAsDouble())
                                        .withTargetDirection(targetHeading));
                } else if (hubRelativeTransform.getMeasureX().in(Inches) <= TOO_CLOSE_DISTANCE_INCHES) {
                        drive.setControl(fieldFacingAngle
                                        .withVelocityX(LinearVelocity.ofRelativeUnits(-0.1, FeetPerSecond))
                                        .withVelocityY(xSupplier.getAsDouble())
                                        .withTargetDirection(targetHeading));
                }
                if (!shot.isValidShot()) {
                        System.out.println("Shot is not valid.");
                        if (hubRelativeTransform.getMeasureX().in(Inches) >= TOO_FAR_DISTANCE_INCHES) {
                                drive.setControl(fieldFacingAngle
                                                .withVelocityX(LinearVelocity.ofRelativeUnits(0.1, FeetPerSecond))
                                                .withVelocityY(xSupplier.getAsDouble())
                                                .withTargetDirection(targetHeading));
                        } else if (hubRelativeTransform.getMeasureX().in(Inches) <= TOO_CLOSE_DISTANCE_INCHES) {
                                drive.setControl(fieldFacingAngle
                                                .withVelocityX(LinearVelocity.ofRelativeUnits(-0.1, FeetPerSecond))
                                                .withVelocityY(xSupplier.getAsDouble())
                                                .withTargetDirection(targetHeading));
                        } else {
                                // Something catostrophic has occurred.
                                System.err.println("Cancelling lock on! Shot invalid");
                                drive.setControl(fieldFacingAngle
                                                .withVelocityX(0)
                                                .withVelocityY(0));
                                fault = true;
                                // this.cancel();
                                return;
                        }
                }
                final double desiredMotorRPM = vRpmSolver.calculateMotorRPM(shot.getFlywheelSpeedMPS());
                // System.out.println("desired rpm = " + desiredMotorRPM);
                if (desiredMotorRPM > shooter.getMaxRPM()) {
                        // Drive forward to get closer to the hub.
                        drive.setControl(fieldFacingAngle
                                        .withVelocityX(LinearVelocity.ofRelativeUnits(0.1, FeetPerSecond))
                                        .withVelocityY(xSupplier.getAsDouble())
                                        .withTargetDirection(targetHeading));

                        return;
                }
                final boolean isShooterReady = vRpmSolver.isReadyToFire();
                // System.out.println("isShooterReady = " + isShooterReady);

                /*
                 * 2. Command the Shooter
                 */
                // shooter.setRPM(desiredMotorRPM);
                // if (isShooterReady) {
                // feeder.setFeederSpeed(1);
                // } else {
                // feeder.setFeederSpeed(0);
                // }

                /*
                 * 3. Command the Drivebase
                 * 
                 * We use the driver's X/Y but OVERRIDE the rotation with our 3D solution.
                 */
                // Correct any input.
                // final double BACKWARDS_CLOCKWISE = 180;
                // final double BACKWARDS_COUNTER_CLOCKWISE = -180;
                // final double turretYaw = MathUtil.inputModulus(shot.getTurretYawDegrees(),
                // BACKWARDS_COUNTER_CLOCKWISE,
                // BACKWARDS_CLOCKWISE);
                final double turretYaw = shot.getTurretYawDegrees();
                // System.out.println("turretYawModulus" + turretYaw);
                // Actually drive.
                fieldFacingAngle
                                .withVelocityX(ySupplier.getAsDouble())
                                .withVelocityY(xSupplier.getAsDouble())
                                .withTargetDirection(targetHeading);
                drive.setControl(fieldFacingAngle);
        }

        // -------------------------------------------------------------------------------------------------------------
        @Override
        public boolean isFinished() {
                if (fault)
                        return true;

                return false;
        }

        // -------------------------------------------------------------------------------------------------------------
        @Override
        public void end(boolean interrupted) {
                System.out.println("In the end.");
                feeder.setFeederSpeed(0);
                shooter.stopShooter();
                fault = false;
        }
}
