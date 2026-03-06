package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Descend extends Command {
    private final Climber climber;

    Descend(final Climber climber) {
        this.climber = climber;

        addRequirements(this.climber);
    }
}
