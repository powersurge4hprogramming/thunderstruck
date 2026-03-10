package frc.robot.commands.climb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.AimCamera;

class ApproachLeft extends Command {
    // =================================================================================================================
    // Private Data Members
    // =================================================================================================================
    private final CommandSwerveDrivetrain drivetrain;
    private final AimCamera aimCamera;
    private final SwerveRequest.FieldCentric fieldDrive;
    private final ChassisSpeeds speed;
    private Distance xFromPost;
    private Distance yFromPost;

    // =================================================================================================================
    // Package Protected Contructor
    // =================================================================================================================
    ApproachLeft(final CommandSwerveDrivetrain drivetrain, final AimCamera aimCamera, final double maxSpeed,
            final double maxAngularRate) {
        this.aimCamera = aimCamera;
        this.drivetrain = drivetrain;
        this.fieldDrive = new SwerveRequest.FieldCentric()
                // Add a 10% deadband
                .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1)
                // Use open-loop control for drive motors
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        speed = new ChassisSpeeds(
                LinearVelocity.ofBaseUnits(0.25, LinearVelocityUnit.combine(Meter, Second)),
                LinearVelocity.ofBaseUnits(0.25, LinearVelocityUnit.combine(Meter, Second)),
                AngularVelocity.ofBaseUnits(0, AngularVelocityUnit.combine(Degrees, Second)));

        this.xFromPost = null;
        this.yFromPost = null;

        addRequirements(this.drivetrain);
    }

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    @Override
    public void execute() {
        final Transform3d leftTag = aimCamera.getTowerRelativeLeftLocation();
        yFromPost = leftTag.getMeasureY();
        xFromPost = leftTag.getMeasureX();
        final boolean isNearLeftPost = yFromPost.isNear(Inches.of(16.125), Inches.of(0.16));

        if (!isNearLeftPost) {
            driveLeft();
        } else {
            driveForward();
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public boolean isFinished() {
        final boolean isNearLeftPost = yFromPost.isNear(Inches.of(16.125), Inches.of(0.16));
        final boolean isNearPostForward = xFromPost.isNear(Inches.of(0), Inches.of(0.16));
        return isNearLeftPost && isNearPostForward;
    }

    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    // =================================================================================================================
    // Private Methods
    // =================================================================================================================
    private void driveLeft() {
        fieldDrive.withVelocityY(speed.vyMetersPerSecond);
        fieldDrive.withVelocityX(0);
    }

    // -----------------------------------------------------------------------------------------------------------------
    private void driveForward() {
        fieldDrive.withVelocityY(0);
        fieldDrive.withVelocityX(speed.vyMetersPerSecond);
    }

}
