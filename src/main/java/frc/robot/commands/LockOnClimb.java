package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.AimCamera;

public class LockOnClimb extends SequentialCommandGroup {
    // =================================================================================================================
    // Sub-Systems
    // =================================================================================================================
    private final Climber climber;
    private final CommandSwerveDrivetrain drivetrain;

    // =================================================================================================================
    // Swerve Drive Configurations
    // =================================================================================================================

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

        addRequirements(climber, drivetrain);

        final Command whatApproachPosition = new InstantCommand(() -> {
            final Transform3d leftTag = aimCamera.getTowerRelativeLeftLocation();
            this.approachedFromLeft = leftTag.getY() > 0;
        });

        addCommands(
                whatApproachPosition);
    }
}
