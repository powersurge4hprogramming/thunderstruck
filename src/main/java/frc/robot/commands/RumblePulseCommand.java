package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumblePulseCommand extends SequentialCommandGroup {
    private final CommandXboxController controller;
    private final double pulseDurationSeconds;
    private final double interPulseDurationSeconds;
    private final double intensity;
    private final byte numPulses;
    private final VibrationSide side;

    RumblePulseCommand(final CommandXboxController controller, final double pulseDurationSeconds,
            final double interPulseDurationSeconds, final double intensity, final byte numPulses,
            final VibrationSide side) {
        this.controller = controller;
        this.pulseDurationSeconds = pulseDurationSeconds;
        this.interPulseDurationSeconds = interPulseDurationSeconds;
        this.intensity = intensity;
        this.numPulses = numPulses;
        this.side = side;
    }
}