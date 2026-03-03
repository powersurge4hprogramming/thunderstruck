package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
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
    // Systems
    // =================================================================================================================
    private final AimCamera aimCamera;

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    public LockOnClimb(final Climber climber, final CommandSwerveDrivetrain drivetrain, final AimCamera aimCamera) {
        this.aimCamera = aimCamera;
        this.climber = climber;
        this.drivetrain = drivetrain;

        addRequirements(climber, drivetrain);
    }

    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public void execute() {
        final Transform3d leftTag = aimCamera.getTowerRelativeLeftLocation();
        final boolean approachedFromLeft = leftTag.getY() > 0;

        // if from the left; move 16.125 inches to the left.
        // then, note how far away in the x direction you are from the left
    }

    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public void end(boolean interrupted) {

    }
}
