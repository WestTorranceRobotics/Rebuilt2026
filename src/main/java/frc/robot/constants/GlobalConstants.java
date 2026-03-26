// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

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
public final class GlobalConstants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OVERRIDE_CONTROLLER_PORT = 1;

        public static final double DEADBAND_THRESHOLD = 0.08;
    }

    public static class PhysicalRobotConstants {
        public static final Mass ROBOT_MASS = Kilograms.of(35);

        public static final Distance kDriveBaseLength = Inches.of(20);
        // TODO: verify width is correct
        public static final Distance kDriveBaseWidth = Inches.of(22.5);

        public static final Distance k_wheelRadius = Inches.of(2);
    }
}
