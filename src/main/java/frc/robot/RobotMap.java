package frc.robot;


/**
 * Mapping and creation of hardware on the robot
 */
public class RobotMap {
    

    /**
     * Inner class to hold limelight physical position offsets
     */
    public static class LimelightOffsets {
        public static final double FLOOR_LIMELIGHT_OFFSET_X = 0;
        public static final double FLOOR_LIMELIGHT_OFFSET_Y = 0;
    }
    /**
     * Inner class to hold CAN ID constants.
     */
    public static class CANID {
        
        // Jan 30/2023:  Swerve Drive CAN IDs match between 2022 and 2023 robot base
        // CAN IDs for Swerve Cancoders
        public static final int LF_CANCODER = 12;
        public static final int RF_CANCODER = 11;
        public static final int LR_CANCODER = 9;
        public static final int RR_CANCODER  = 10;
        // CAN IDs for Steer Motors
        public static final int LF_STEER_MOTOR = 4;
        public static final int RF_STEER_MOTOR = 6;
        public static final int LR_STEER_MOTOR = 2;
        public static final int RR_STEER_MOTOR = 8;
        // CAN IDs for Drive Motors
        public static final int LF_DRIVE_MOTOR = 3;
        public static final int RF_DRIVE_MOTOR = 5;
        public static final int LR_DRIVE_MOTOR = 1;
        public static final int RR_DRIVE_MOTOR = 7;
        // CAN ID for ARM
        public static final int ARM_MOTOR = 13;
        public static final int ARM_CANCODER = 14;
        // CAN ID for Grabber
        public static final int GRABBER_MOTOR = 15;
        // CAN ID for CTR Pigeon Gyro
        public static final int PIGEON = 20;
    }

    /**
     * Inner class to hold RoboRIO I/O connection constants
     */
  public static class RIO {
        // Grabber - Input Digital Switch (true if item is grabbed)
        public static final int DIO_0 = 0;


        
        /** Pin of Analog Input connection 1 */
        public static final int ANALOG_0 = 0;
        
        /** CAN ID of left talon 1. */
        //public static final int LEFT_CONTROLLER_1 = 0;
        /** CAN ID of left talon 2. */
        //public static final int LEFT_CONTROLLER_2 = 1;
        
        /** CAN ID of right talon 1. */
        //public static final int RIGHT_CONTROLLER_1 = 2;
        /** CAN ID of right talon 2. */
        //public static final int RIGHT_CONTROLLER_2 = 3;
    }

    public static class PWMPorts {
        /** PWM Port for led strip */
        public static final int LED_BLINKIN = 0;

        // PWM port for camera tilting subsystem
        public static final int CAMERA_SERVO_ID = 1;
    } 

}
