package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FaceWall extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentricFacingAngle fieldFacingAngle;

    FaceWall(final double maxSpeed, final double maxAngularRate, final CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.fieldFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
                // Add a 10% deadband
                .withDeadband(maxSpeed * 0.1)
                // Use open-loop control for drive motors
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(this.drivetrain);
    }

    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public void execute() {
        // Face the wall.
        final Optional<Alliance> allianceOptional = DriverStation.getAlliance();
        final Alliance alliance = allianceOptional
                .orElseThrow(() -> new RuntimeException("Whaaaaa... No alliance is set?"));

        Rotation2d targetHeading = (alliance == Alliance.Blue) ? Rotation2d.fromDegrees(180.0)
                : Rotation2d.fromDegrees(0.0);
        drivetrain.setControl(fieldFacingAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(targetHeading));
    }

}
