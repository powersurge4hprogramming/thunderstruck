package frc.robot;

/**
 * {@summary}
 * This holds all of the static constant values on the CANBus that we are using.
 */
public final class CANBus {
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
            public static final byte LEADER = 19;
            public static final byte FOLLOWER = 18;
        }

        /**
         * {@summary}
         * The IDs for the {@link frc.robot.subsystems.Feeder Feeder}.
         */
        public static final class FEEDER {
            public static final byte MOTOR = 2;
        }

        /**
         * {@summary}
         * The IDs for the {@link frc.robot.subsystems.Collector Collector}.
         */
        public static final class COLLECTOR {
            public static final byte MOTOR = 1;
        }

        /**
         * {@summary}
         * The IDs for the {@link frc.robot.subsystems.Collector Collector}.
         */
        public static final class AGGITATOR {
            public static final byte MOTOR = 4;
        }

        /**
         * {@summary}
         * The IDs for the {@link frc.robot.subsystems.Climber Climber}.
         */
        public static final class CLIMBER {
            // TODO: Need a real value here.
            public static final byte MOTOR = 5;
        }

        /**
         * {@summary}
         * The ID for the
         * {@link edu.wpi.first.wpilibj.PowerDistribution.PowerDistribution
         * PowerDistribution} module.
         */
        public static final byte POWER_DISTRIBUTION = 3;
    }
}
