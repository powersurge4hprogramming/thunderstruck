package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.AimCamera;

public class LockOnClimb extends SequentialCommandGroup {
    // =================================================================================================================
    // Private Data Members
    // =================================================================================================================
    private boolean approachedFromLeft;

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    public LockOnClimb(final Climber climber, final CommandSwerveDrivetrain drivetrain, final AimCamera aimCamera,
            final double maxSpeed, final double maxAngularRate) {
        addRequirements(climber, drivetrain);

        final Command whatApproachPosition = new InstantCommand(() -> {
            final Transform3d leftTag = aimCamera.getTowerRelativeLeftLocation();
            approachedFromLeft = leftTag.getY() > 0;
        });

        addCommands(
                whatApproachPosition,
                new FaceWall(maxSpeed, maxAngularRate, drivetrain),
                new ConditionalCommand(
                        new ApproachLeft(drivetrain, aimCamera, maxSpeed, maxAngularRate),
                        new ApproachRight(drivetrain, aimCamera, maxSpeed, maxAngularRate),
                        () -> approachedFromLeft),
                new Ascend(climber),
                new WaitCommand(2),
                new Descend(climber));
    }
}
