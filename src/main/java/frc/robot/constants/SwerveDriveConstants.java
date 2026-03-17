package frc.robot.constants;

public final class SwerveDriveConstants {
    public static class RealRobotConstants {
        public static final int PIGEON2_ID = 9;

        public static final int FR_DRIVE_MOTOR_ID = 33;
        public static final int FL_DRIVE_MOTOR_ID = 52;
        public static final int BL_DRIVE_MOTOR_ID = 34;
        public static final int BR_DRIVE_MOTOR_ID = 54;

        public static final int FR_AZIMUTH_MOTOR_ID = 11;
        public static final int FL_AZIMUTH_MOTOR_ID = 14;
        public static final int BL_AZIMUTH_MOTOR_ID = 13;
        public static final int BR_AZIMUTH_MOTOR_ID = 12;

        public static final int FR_CANCODER_ID = 31;
        public static final int FL_CANCODER_ID = 34;
        public static final int BL_CANCODER_ID = 60;
        public static final int BR_CANCODER_ID = 33;

        public static final boolean DRIVE_INVERTED = false;
        public static final boolean AZIMUTH_INVERTED = true;

        // TODO zero these encoders
        public static final double FR_CANCODER_OFFSET = -0.43896484375;
        public static final double FL_CANCODER_OFFSET = -0.03564453125;
        public static final double BL_CANCODER_OFFSET =0.493896484375;
        public static final double BR_CANCODER_OFFSET =  -0.037353515625;

        public static final double kPDrive = 0.098616;
        public static final double kIDrive = 0;
        public static final double kDDrive = 0;
        public static final double kSDrive = 0.0058039;
        public static final double kVDrive = 0.11504;

        public static final double kPAzimuth = 2;
        public static final double kIAzimuth = 0;
        public static final double kDAzimuth = 0;

        public static final double kPTranslation = 8;
        public static final double kITranslation = 3;
        public static final double kDTranslation = 0;

        public static final double kPRotation = 4;
        public static final double kIRotation = 8;
        public static final double kDRotation = 0.3;

        public static final double MAX_ANGULAR_SPEED = 0.05 * Math.PI;
        public static final double PROPORTIONALITY_CONSTANT = 0.35; // TODO tune this value
    }

    public static class SimulatedControlSystemConstants {
        public static final double kSDrive = 0;
        public static final double kVDrive = 2.435;
        public static final double kADrive = 0;

        public static final double kPDrive = 8;
        public static final double kIDrive = 0;
        public static final double kDDrive = 0;

        public static final double kPSteer = 13.26;
        public static final double kISteer = 0;
        public static final double kDSteer = 0.59364;
    }
}
