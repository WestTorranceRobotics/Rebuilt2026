package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
    public static final int FEEDER_MOTOR_ID = 49;
    public static final int LAUNCHER_MOTOR_1_ID = 22;
    public static final int LAUNCHER_MOTOR_2_ID = 3;

    // current limit values for shooter motors
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;
    // FIXME these current limits are all way too high

    public static final double LATENCY_COMPENSATION = 0; // TODO tune latency compensation

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TOF_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap SHOOTER_MAP = new InterpolatingDoubleTreeMap();

    static {
        // TODO build lookup tables
        DISTANCE_TO_TOF_MAP.put(1.5, 0.42);
        DISTANCE_TO_TOF_MAP.put(2.0, 0.51);
        DISTANCE_TO_TOF_MAP.put(2.5, 0.58);
        DISTANCE_TO_TOF_MAP.put(3.0, 0.65);
        DISTANCE_TO_TOF_MAP.put(3.5, 0.71);
        DISTANCE_TO_TOF_MAP.put(4.0, 0.78);
        DISTANCE_TO_TOF_MAP.put(4.5, 0.84);
        DISTANCE_TO_TOF_MAP.put(5.0, 0.91);

        SHOOTER_MAP.put(3.57142857, 2800.0);
        SHOOTER_MAP.put(3.92156863, 3100.0);
        SHOOTER_MAP.put(4.31034483, 3400.0);
        SHOOTER_MAP.put(4.61538462, 3650.0);
        SHOOTER_MAP.put(4.92957746, 3900.0);
        SHOOTER_MAP.put(5.12820513, 4100.0);
        SHOOTER_MAP.put(5.35714286, 4350.0);
        SHOOTER_MAP.put(5.49450549, 4550.0);
    }
}
