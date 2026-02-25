package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumblePulseCommand extends SequentialCommandGroup {
    // =================================================================================================================
    // Public Methods
    // =================================================================================================================
    public RumblePulseCommand(final CommandXboxController controller, final double pulseDurationSeconds,
            final double interPulseDurationSeconds, final double intensity, final byte numPulses,
            final RumbleType side) {
        for (int i = 0; i < numPulses; i++) {
            addCommands(
                    new InstantCommand(() -> controller.setRumble(side, intensity)),
                    new WaitCommand(pulseDurationSeconds),
                    new InstantCommand(() -> controller.setRumble(side, 0.0)));
            if (i < numPulses - 1) {
                addCommands(new WaitCommand(interPulseDurationSeconds));
            }
        }
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
    public static RumblePulseCommand createLongSinglePulse(final CommandXboxController controller,
            final double intensity, final RumbleType side) {
        final byte numPulses = 1;
        final double pulseDurationSeconds = 0.4;
        final double interPulseDurationSeconds = 0;
        return new RumblePulseCommand(controller, pulseDurationSeconds, interPulseDurationSeconds, intensity, numPulses,
                side);
    }

    // -----------------------------------------------------------------------------------------------------------------
    public static RumblePulseCommand createShortDoublePulse(final CommandXboxController controller,
            final double intensity, final RumbleType side) {
        final byte numPulses = 2;
        final double pulseDurationSeconds = 0.1;
        final double interPulseDurationSeconds = 0.3;
        return new RumblePulseCommand(controller, pulseDurationSeconds, interPulseDurationSeconds, intensity, numPulses,
                side);
    }

    // -----------------------------------------------------------------------------------------------------------------
    public static RumblePulseCommand createLongDoublePulse(final CommandXboxController controller,
            final double intensity, final RumbleType side) {
        final byte numPulses = 2;
        final double pulseDurationSeconds = 0.4;
        final double interPulseDurationSeconds = 0.5;
        return new RumblePulseCommand(controller, pulseDurationSeconds, interPulseDurationSeconds, intensity, numPulses,
                side);
    }

    // -----------------------------------------------------------------------------------------------------------------
    public static WaitCommand createShortWaitCommand() {
        return new WaitCommand(0.1);
    }

    // -----------------------------------------------------------------------------------------------------------------
    public static WaitCommand createLongWaitCommand() {
        return new WaitCommand(0.2);
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * {@summary}
     * I think we need to override the super class's
     * {@link SequentialCommandGroup#getInterruptionBehavior()}
     */
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}