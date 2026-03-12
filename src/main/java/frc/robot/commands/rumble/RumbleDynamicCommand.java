package frc.robot.commands.rumble;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleDynamicCommand extends Command {
    // =================================================================================================================
    // Private Data Members
    // =================================================================================================================
    private final CommandXboxController controller;
    private final DoubleSupplier intensity;
    private final Supplier<RumbleType> side;

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    // TODO: Don't pass in a Supplier<RumbleType>, but instead pass in a RumbleType.
    public RumbleDynamicCommand(final CommandXboxController controller, final DoubleSupplier intensitySupplier,
            final Supplier<RumbleType> side) {
        this.controller = controller;
        intensity = intensitySupplier;
        this.side = side;
    }

    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public void initialize() {
        controller.setRumble(side.get(), intensity.getAsDouble());
    }

    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public void execute() {
        controller.setRumble(side.get(), intensity.getAsDouble());
    }

    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public void end(boolean interrupted) {
        controller.setRumble(side.get(), 0.0);
    }
}
