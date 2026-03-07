package frc.robot.commands.climb;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.AimCamera;

public class LockOnClimb extends SequentialCommandGroup {
    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    public LockOnClimb(final Climber climber, final CommandSwerveDrivetrain drivetrain, final AimCamera aimCamera,
            final double maxSpeed, final double maxAngularRate) {
        addRequirements(climber, drivetrain);

        addCommands(
                new FaceWallForward(maxSpeed, maxAngularRate, drivetrain),
                new ConditionalCommand(
                        new ApproachLeft(drivetrain, aimCamera, maxSpeed, maxAngularRate),
                        new ApproachRight(drivetrain, aimCamera, maxSpeed, maxAngularRate),
                        () -> {
                            final Transform3d leftTag = aimCamera.getTowerRelativeLeftLocation();
                            return leftTag.getY() > 0;
                        }),
                new FaceWallRear(maxSpeed, maxAngularRate, drivetrain),
                climber.upward(),
                new WaitCommand(2),
                climber.downward());
    }
}
