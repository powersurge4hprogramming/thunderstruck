package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.AimCamera;

public class LockOnClimb extends Command {
    // =================================================================================================================
    // Sub-Systems
    // =================================================================================================================
    private final Climber climber;
    private final CommandSwerveDrivetrain drivetrain;

    // =================================================================================================================
    // Swerve Drive Configurations
    // =================================================================================================================
    private final SwerveRequest.FieldCentric fieldDrive;

    // =================================================================================================================
    // Systems
    // =================================================================================================================
    private final AimCamera aimCamera;

    // =================================================================================================================
    // Systems
    // =================================================================================================================
    private boolean approachedFromLeft;

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    public LockOnClimb(final Climber climber, final CommandSwerveDrivetrain drivetrain, final AimCamera aimCamera,
            final double maxSpeed, final double maxAngularRate) {
        this.aimCamera = aimCamera;
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.fieldDrive = new SwerveRequest.FieldCentric()
                // Add a 10% deadband
                .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1)
                // Use open-loop control for drive motors
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(climber, drivetrain);
    }

    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public void initialize() {
        final Transform3d leftTag = aimCamera.getTowerRelativeLeftLocation();
        this.approachedFromLeft = leftTag.getY() > 0;
    }

    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public void execute() {
        final Transform3d leftTag = aimCamera.getTowerRelativeLeftLocation();

        if (approachedFromLeft) {
            executeApproachFromLeft(leftTag);
        } else {
            final Transform3d rightTag = aimCamera.getTowerRelativeRightLocation();
            executeApproachFromRight(rightTag);
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    private void executeApproachFromLeft(final Transform3d leftTag) {
        if (leftTag.getMeasureY().isNear(Meter.of(16.125), Meter.of(0.01))) {
            // move 16.125 inches to the left.
            final ChassisSpeeds speed = new ChassisSpeeds(
                    LinearVelocity.ofBaseUnits(0.25, LinearVelocityUnit.combine(Meter, Second)),
                    LinearVelocity.ofBaseUnits(0.25, LinearVelocityUnit.combine(Meter, Second)),
                    AngularVelocity.ofBaseUnits(0, AngularVelocityUnit.combine(Degrees, Second)));
            fieldDrive.withVelocityX(speed.vyMetersPerSecond);
            // then, note how far away in the x direction you are from the left
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    private void executeApproachFromRight(final Transform3d rightTag) {
    }

    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public void end(boolean interrupted) {

    }
}
