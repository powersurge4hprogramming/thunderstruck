package frc.robot.commands;

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

public class ApproachLeft extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final AimCamera aimCamera;
    private final SwerveRequest.FieldCentric fieldDrive;

    ApproachLeft(final CommandSwerveDrivetrain drivetrain, final AimCamera aimCamera, final double maxSpeed,
            final double maxAngularRate) {
        this.aimCamera = aimCamera;
        this.drivetrain = drivetrain;
        this.fieldDrive = new SwerveRequest.FieldCentric()
                // Add a 10% deadband
                .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1)
                // Use open-loop control for drive motors
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    }

    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public void execute() {
        final Transform3d leftTag = aimCamera.getTowerRelativeLeftLocation();
        final Distance leftPostFromLeftTag = Inches.of(16.125);
        final boolean isNearLeftPost = leftTag.getMeasureY().isNear(leftPostFromLeftTag, Inches.of(0.16));
        if (!isNearLeftPost) {
            final ChassisSpeeds speed = new ChassisSpeeds(
                    LinearVelocity.ofBaseUnits(0.25, LinearVelocityUnit.combine(Meter, Second)),
                    LinearVelocity.ofBaseUnits(0.25, LinearVelocityUnit.combine(Meter, Second)),
                    AngularVelocity.ofBaseUnits(0, AngularVelocityUnit.combine(Degrees, Second)));
            fieldDrive.withVelocityX(speed.vyMetersPerSecond);
        } else {
            // Ok, I am now in front of the left post. Let's do this!
            // Note how far away in the x direction you are from the left
            final Distance xFromPost = leftTag.getMeasureX();
            // Rotate 180 degrees to face the climber at the post.
        }
    }
}
