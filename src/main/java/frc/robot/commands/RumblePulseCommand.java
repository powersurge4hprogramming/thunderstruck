package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumblePulseCommand extends SequentialCommandGroup {
    // =================================================================================================================
    // Private Data Members
    // =================================================================================================================
    private final CommandXboxController controller;
    private final double pulseDurationSeconds;
    private final double interPulseDurationSeconds;
    private final double intensity;
    private final byte numPulses;
    private final RumbleType side;

    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    public RumblePulseCommand(final CommandXboxController controller, final double pulseDurationSeconds,
            final double interPulseDurationSeconds, final double intensity, final byte numPulses,
            final RumbleType side) {
        this.controller = controller;
        this.pulseDurationSeconds = pulseDurationSeconds;
        this.interPulseDurationSeconds = interPulseDurationSeconds;
        this.intensity = intensity;
        this.numPulses = numPulses;
        this.side = side;
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * {@summary}
     * This method is a <b>factory</b> method: it's sole purpose is to create a
     * pre-configured <code>RumblePulseCommand</code>. This particular one is for a
     * short, single, vibration pulse of the controller.
     * 
     * @apiNote Usage example:
     *          <code>RumblePulseCommand.createShortSinglePulse(controller, 0.5, RumbleType.kRightRumble);</code>
     * 
     * @param controller The controller to vibrate.
     * @param intensity  The intensity of the vibration from 0 to 1.
     * @param side       The side at which to vibrate the controller.
     * @return The pre-configured vibration command with the given parameters.
     */
    public static RumblePulseCommand createShortSinglePulse(final CommandXboxController controller,
            final double intensity, final RumbleType side) {
        final byte numPulses = 1;
        final double pulseDurationSeconds = 0.1;
        final double interPulseDurationSeconds = 0;
        return new RumblePulseCommand(controller, pulseDurationSeconds, interPulseDurationSeconds, intensity, numPulses,
                side);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // TODO: Our long pulse command goes here.
}