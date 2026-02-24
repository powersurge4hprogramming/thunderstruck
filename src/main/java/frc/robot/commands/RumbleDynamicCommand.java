package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleDynamicCommand {
    // =================================================================================================================
    // Private Data Members
    // =================================================================================================================
    private final CommandXboxController controller;
    private final DoubleSupplier intensity;
    private final RumbleType side;

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    public RumbleDynamicCommand(final CommandXboxController controller, final DoubleSupplier intensitySupplier,
            final RumbleType side) {
        this.controller = controller;
        intensity = intensitySupplier;
        this.side = side;
    }
}
