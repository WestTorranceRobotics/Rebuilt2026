package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
    public static final int feederMotorID = 1;
    public static final int firstIntakeMotorID = 2;
    public static final int secondIntakeMotorID = 3;
    public static final int thirdIntakeMotorID = 4; // FIXME put in proper IDs

    // current limit values for shooter motors
    public static final int feederMotorCurrentLimit = 60;
    public static final int launcherMotorCurrentLimit = 60;
    // FIXME these current limits are all way too high

    public static final double latencyCompensation = 0; // TODO tune latency comepnsation

    public static final InterpolatingDoubleTreeMap distanceToTOFMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

    static {
        // TODO build lookup tables
        distanceToTOFMap.put(1.5, 0.42);
        distanceToTOFMap.put(2.0, 0.51);
        distanceToTOFMap.put(2.5, 0.58);
        distanceToTOFMap.put(3.0, 0.65);
        distanceToTOFMap.put(3.5, 0.71);
        distanceToTOFMap.put(4.0, 0.78);
        distanceToTOFMap.put(4.5, 0.84);
        distanceToTOFMap.put(5.0, 0.91);

        shooterMap.put(3.57142857, 2800.0);
        shooterMap.put(3.92156863, 3100.0);
        shooterMap.put(4.31034483, 3400.0);
        shooterMap.put(4.61538462, 3650.0);
        shooterMap.put(4.92957746, 3900.0);
        shooterMap.put(5.12820513, 4100.0);
        shooterMap.put(5.35714286, 4350.0);
        shooterMap.put(5.49450549, 4550.0);
    }
}
