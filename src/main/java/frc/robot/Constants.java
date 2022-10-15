package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * 
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * Phisical Connections and Propertys.
     */
    public static class Hardware {
        public static final int DRIVER_CONTROLER_PORT = 0;
        public static final double CONTROLER_DEADBAND = 0.1;
        public static final int CTRE_MAG_ENCODER_CPR = 2048;
        public static final int NEO_CPR = 42;
        public static final double DRIVE_GEER_RATIO = 8.16;
        public static final double TURN_GEER_RATIO = 12.8;
        public static final double WHEEL_DIAMETER_INCHES = 4;
        public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(Math.PI * WHEEL_DIAMETER_INCHES);

        public static final double TRACK_WIDTH_METER = Units.inchesToMeters(18.0);
        public static final double WHEEL_BASE_METER = Units.inchesToMeters(17.125);
        public static final double MAX_SPEED_METER_PER_SECOND = 5.0;
        public static final int PIDEON_CAN_ID = 6;
    }

    /**
     * Propertys of the Swerve Modules.
     */
    public static class SwerveModuleConstatntes {

        public static final int SLOT_ID = 0;
        public static final double TURN_KP = 1.0;
        public static final double TURN_KI = 0.0;
        public static final double TURN_KD = 0.75;
        
        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.01;
        public static final double DRIVE_KF = 0.245;

        public static final String[] LABLES = new String[] { "Front Left", "Front Right", "Rear Left", "Rear Right" };
        public static final int[] TURN_CAN_IDS = new int[] { 13, 2, 14, 1 };
        public static final int[] DRIVE_CAN_IDS = new int[] { 12, 3, 15, 20 };
        public static final int[] ABS_ENCODER_DIO_PORT = new int[] { 24, 8, 5, 2 };
        public static final boolean[] DRIVE_ENCODER_REVERSED = new boolean[] { false, true, false, true };
        public static final double[] HOMES_RAD = { 1.52, 1.48, 4.89, 0.47 };
        public static final Translation2d[] POSITIONS = new Translation2d[] {
            new Translation2d(Hardware.WHEEL_BASE_METER / 2, Hardware.TRACK_WIDTH_METER / 2),
            new Translation2d(Hardware.WHEEL_BASE_METER / 2, -Hardware.TRACK_WIDTH_METER / 2),
            new Translation2d(-Hardware.WHEEL_BASE_METER / 2, Hardware.TRACK_WIDTH_METER / 2),
            new Translation2d(-Hardware.WHEEL_BASE_METER / 2, -Hardware.TRACK_WIDTH_METER / 2) };    
    }

    /**
     * .
     */
    public static class SwerveControlConstants {
        public static final double ROTATION_KP = 1.0;
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;
    }

    /**
     * Driver controler constants/settings.
     */
    public static class Driver {
        public static final double MAX_STRAFE_SPEED = 5;
        public static final double MAX_TURN_SPEED = 3 * Math.PI;
    }
}
