package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.AimCamera;

public class ApproachRight extends Command {
    // =================================================================================================================
    // Private Data Members
    // =================================================================================================================
    private final CommandSwerveDrivetrain drivetrain;
    private final AimCamera aimCamera;
    private final SwerveRequest.FieldCentric fieldDrive;

    // =================================================================================================================
    // Package Protected Contructor
    // =================================================================================================================
    ApproachRight(final CommandSwerveDrivetrain drivetrain, final AimCamera aimCamera, final double maxSpeed,
            final double maxAngularRate) {
        this.aimCamera = aimCamera;
        this.drivetrain = drivetrain;
        this.fieldDrive = new SwerveRequest.FieldCentric()
                // Add a 10% deadband
                .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1)
                // Use open-loop control for drive motors
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(this.drivetrain);
    }

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
}
