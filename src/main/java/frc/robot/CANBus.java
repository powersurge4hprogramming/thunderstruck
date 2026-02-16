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
            public static final byte LEADER = 0;
            public static final byte FOLLOWER = 0;
            public static final byte LOADER = 0
        }

        /**
         * {@summary}
         * The IDs for the {@link frc.robot.subsystems.Collector Collector}.
         */
        public static final class COLLECTOR {
            // TODO: Need a real value here.
            public static final byte MOTOR = 0;
        }

        /**
         * {@summary}
         * The IDs for the {@link frc.robot.subsystems.Climber Climber}.
         */
        public static final class CLIMBER {
            // TODO: Need a real value here.
            public static final byte MOTOR = 0;
        }

        /**
         * {@summary}
         * The ID for the
         * {@link edu.wpi.first.wpilibj.PowerDistribution.PowerDistribution
         * PowerDistribution} module.
         */
        // TODO: Need a real value here.
        public static final byte POWER_DISTRIBUTION = 0;
    }
}
