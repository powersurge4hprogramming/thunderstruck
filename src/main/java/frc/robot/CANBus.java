package frc.robot;

/**
 * {@summary}
 * This holds all of the static constant values on the CANBus that we are using.
 */
public class CANBus {
    // =================================================================================================================
    // Buses
    // =================================================================================================================
    /**
     * {@summary}
     * This holds all of the CANBus names.
     */
    public static final class BUS {
        public static final String RIO = com.ctre.phoenix6.CANBus.roboRIO().getName();
        public static final String CANIVORE = "canivore";
    }

    // =================================================================================================================
    // IDs
    // =================================================================================================================
    /**
     * {@summary}
     * This holds all of the CANBus IDs.
     */
    public static final class ID {
        /**
         * {@summary}
         * The IDs for the {@link frc.robot.subsystems.Shooter Shooter}.
         */
        public static final class SHOOTER {
            public static final short LEADER = 0;
            public static final short FOLLOWER = 0;
        }
    }
}
