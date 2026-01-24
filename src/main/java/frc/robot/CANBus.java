package frc.robot;

public class CANBus {
    // =================================================================================================================
    // Buses
    // =================================================================================================================
    public static final class BUS {
        public static final String RIO = com.ctre.phoenix6.CANBus.roboRIO().getName();
        public static final String CANIVORE = "canivore";
    }

    // =================================================================================================================
    // IDs
    // =================================================================================================================
    public static final class ID {
        public static final class SHOOTER {
            public static final short LEADER = 0;
            public static final short FOLLOWER = 0;
        }
    }
}
