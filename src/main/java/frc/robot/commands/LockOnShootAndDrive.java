package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.AimCamera;
import frc.robot.physics.ballistics.VelocityAngleSolver;
import frc.robot.physics.ballistics.VelocityAngleSolver.ShotResult;
import frc.robot.physics.rotational.VelocityToRPMSolver;

public class LockOnShootAndDrive extends Command {
    // =================================================================================================================
    // Sub-Systems
    // =================================================================================================================
    private final Shooter shooter;
    private final CommandSwerveDrivetrain drive;

    // =================================================================================================================
    // Systems
    // =================================================================================================================
    private final AimCamera aimCamera;

    // =================================================================================================================
    // Data Suppliers
    // =================================================================================================================
    // Allow driver to still move X/Y
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    // =================================================================================================================
    // Solvers
    // =================================================================================================================
    private final VelocityAngleSolver vaSolver;
    private final VelocityToRPMSolver vRpmSolver;

    // =================================================================================================================
    // Swerve Drive Configurations
    // =================================================================================================================
    final SwerveRequest.FieldCentricFacingAngle fieldFacingAngle;

    public LockOnShootAndDrive(final Shooter shooter, final CommandSwerveDrivetrain drive, final AimCamera aimCamera,
            final DoubleSupplier xMove, final DoubleSupplier yMove, final DoubleSupplier batteryVoltageSupplier,
            final double MaxSpeed) {
        this.shooter = shooter;
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
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // REQUIRE BOTH: This stops any other drive or shooter commands
        addRequirements(shooter, drive);
    }

    @Override
    public void execute() {
        // 1. One physics calculation to rule them all
        ChassisSpeeds robotCentric = drive.getState().Speeds;
        Rotation2d heading = drive.getState().Pose.getRotation();
        // Convert to Field Relative automatically
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(robotCentric, heading);
        double fieldVx = fieldRelative.vxMetersPerSecond; // x in field frame (blue origin forward)
        double fieldVy = fieldRelative.vyMetersPerSecond; // y in field frame
        final Transform3d hubRelativeTransform = aimCamera.getHubRelativeLocation();

        if (hubRelativeTransform == null) {
            // TODO: Need a way to handle outside state, or something to handle it.
            this.cancel();
        }

        final double shapeScalar = 1;
        final boolean isBlocked = false;

        final ShotResult shot = vaSolver.calculate(hubRelativeTransform, heading, fieldVx, fieldVy, shapeScalar,
                isBlocked);

        final double motorRPM = vRpmSolver.calculateMotorRPM(shot.getFlyWheelSpeedMPS());
        // 2. Command the Shooter
        shooter.setRPM(motorRPM);
        shooter.setLaunchAngle(shot.getHoodPitchDegrees());

        // 3. Command the Drivebase
        // We use the driver's X/Y but OVERRIDE the rotation with our 3D solution
        final double BACKWARDS_CLOCKWISE = 180;
        final double BACKWARDS_COUNTER_CLOCKWISE = -180;
        final double turretYaw = MathUtil.inputModulus(shot.getTurretYawDegrees(), BACKWARDS_COUNTER_CLOCKWISE,
                BACKWARDS_CLOCKWISE);
        drive.setControl(fieldFacingAngle
                .withVelocityX(xSupplier.getAsDouble())
                .withVelocityY(ySupplier.getAsDouble())
                .withTargetDirection(new Rotation2d(turretYaw)));
    }
}