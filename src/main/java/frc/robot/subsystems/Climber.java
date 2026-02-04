package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.DigitalInputBus;

public class Climber extends SubsystemBase {
    private final DigitalInput leftArmLimit;
    private final DigitalInput rightARmLimit;

    // -----------------------------------------------------------------------------------------------------------------
    public Climber() {
        leftArmLimit = new DigitalInput(DigitalInputBus.CLIMBER.LEFT_ARM_LIMIT_SWITCH);
        rightARmLimit = new DigitalInput(DigitalInputBus.CLIMBER.RIGHT_ARM_LIMIT_SWITCH);
    }

    /**
     * {@summary}
     * Tell the climber to go upward.
     * 
     * @return The {@link Command} to perform the action.
     */
    public Command upward() {
        throw new RuntimeException("Not implemented yet.");
    }

    /**
     * {@summary}
     * Tell the climber to go downward.
     * 
     * @return The {@link Command} to perform the action.
     */
    public Command downward() {
        throw new RuntimeException("Not implemented yet.");
    }
}
