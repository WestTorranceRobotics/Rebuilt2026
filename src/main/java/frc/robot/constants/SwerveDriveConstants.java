package frc.robot.constants;

public final class SwerveDriveConstants {
    public static class RealRobotConstants {
        public static final int kPigeon2ID = 9;

        public static final int kFRDriveMotorID = 52;
        public static final int kFLDriveMotorID = 33;
        public static final int kBLDriveMotorID = 34;
        public static final int kBRDriveMotorID = 54;

        public static final int kFRAzimuthMotorID = 11;
        public static final int kFLAzimuthMotorID = 14;
        public static final int kBLAzimuthMotorID = 13;
        public static final int kBRAzimuthMotorID = 12;

        public static final int kFRCANCoderID = 31;
        public static final int kFLCANCoderID = 34;
        public static final int kBLCANCoderID = 60;
        public static final int kBRCANCoderID = 33;

        public static final boolean kDriveReversed = false;
        public static final boolean kAzimuthReversed = true;

        // TODO zero these encoders
        public static final double kFRCANCoderOffset = 0.184814453125;
        public static final double kFLCANCoderOffset = 0.357421875;
        public static final double kBLCANCoderOffset = 0.099609375;
        public static final double kBRCANCoderOffset = 0.333251953125;

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

        public static final double kMaxAngularSpeed = 0.05 * Math.PI;
        public static final double kConstantOfProportionality = 0.35; // TODO tune this value
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
