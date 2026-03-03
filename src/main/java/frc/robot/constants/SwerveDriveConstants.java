package frc.robot.constants;

public final class SwerveDriveConstants {
    public static class RealRobotConstants {
        public static final int kPigeon2ID = 9;

        public static final int kFRDriveMotorID = 33;
        public static final int kFLDriveMotorID = 34;
        public static final int kBLDriveMotorID = 52;
        public static final int kBRDriveMotorID = 54;

        public static final int kFRAzimuthMotorID = 13;
        public static final int kFLAzimuthMotorID = 14;
        public static final int kBLAzimuthMotorID = 12;
        public static final int kBRAzimuthMotorID = 11;

        public static final int kFRCANCoderID = 34;
        public static final int kFLCANCoderID = 60;
        public static final int kBLCANCoderID = 31;
        public static final int kBRCANCoderID = 33;

        public static final boolean kDriveReversed = false;
        public static final boolean kAzimuthReversed = true;

        // TODO zero these encoders
        public static final double kFRCANCoderOffset = 0.11474609375;
        public static final double kFLCANCoderOffset = -0.142578125;
        public static final double kBLCANCoderOffset = 0.09033203125;
        public static final double kBRCANCoderOffset = -0.07421875;

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
