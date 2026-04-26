package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;

public class VisionConstants {
    public static final String CAMERA_NAME = "Arducam_OV9281_USB_Camera";

    public static final Transform3d ROBOT_TO_CAM = new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0)); // TODO: Update these values to match where the camera is

    public static final List<Integer> NEUTRAL_ZONE_APRILTAG_IDS = List.of(6, 4, 3, 1, 17, 19, 20, 22);

    public static final double BASE_STD_DEV = 0.1;
    public static final double DISTANCE_WEIGHT = 0.1;
    public static final double AMBIGUITY_WEIGHT = 10.0;
}
