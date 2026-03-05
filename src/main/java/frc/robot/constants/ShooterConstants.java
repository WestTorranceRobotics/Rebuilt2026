package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public final class ShooterConstants {
    public static final int feederMotorID = 1;
    public static final int firstIntakeMotorID = 2;
    public static final int secondIntakeMotorID = 3;
    public static final int thirdIntakeMotorID = 4; // FIXME put in proper IDs

    // current limit values for shooter motors
    public static final int feederMotorCurrentLimit = 60;
    public static final int launcherMotorCurrentLimit = 60;
    // FIXME these current limits are all way too high

    public static final double latencyCompensation = 0.1; // TODO tune latency comepnsation

    public static final InterpolatingTreeMap<Double, Double> distanceToTOFMap = new InterpolatingTreeMap<>(null, null);
    public static final InterpolatingTreeMap<Double, Double> shooterMap = new InterpolatingTreeMap<>(null, null);

    static {
        // TODO build lookup tables
        distanceToTOFMap.put(100.0, 3.0);
        distanceToTOFMap.put(120.0, 3.5);
        distanceToTOFMap.put(140.0, 4.0);
        distanceToTOFMap.put(160.0, 4.5);
        distanceToTOFMap.put(180.0, 5.0);
        distanceToTOFMap.put(200.0, 5.5);

        shooterMap.put(33.33333333, 3000.0);
        shooterMap.put(34.28571429, 3200.0);
        shooterMap.put(35.0, 3400.0);
        shooterMap.put(35.55555556, 3600.0);
        shooterMap.put(36.0, 3800.0);
        shooterMap.put(36.36363636, 4000.0);
    }
}
