package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleDynamicCommand {
    // =================================================================================================================
    // Private Data Members
    // =================================================================================================================
    private final CommandXboxController controller;
    private final DoubleSupplier intensity;
    private final VibrationSide side;

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    public RumbleDynamicCommand(final CommandXboxController controller, final DoubleSupplier intensitySupplier,
            final VibrationSide side) {
        this.controller = controller;
        intensity = intensitySupplier;
        this.side = side;
    }
}
