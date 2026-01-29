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
        public static final com.ctre.phoenix6.CANBus RIO = com.ctre.phoenix6.CANBus.roboRIO();
        public static final com.ctre.phoenix6.CANBus CANIVORE = new com.ctre.phoenix6.CANBus("canivore");
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
            // TODO: Need real values here.
            public static final short LEADER = 0;
            public static final short FOLLOWER = 0;
        }

        /**
         * {@summary}
         * The IDs for the {@link frc.robot.subsystems.Collector Collector}.
         */
        public static final class COLLECTOR {
            // TODO: Need a real value here.
            public static final short MOTOR = 0;
        }
    }
}
