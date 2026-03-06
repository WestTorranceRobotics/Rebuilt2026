package frc.robot.subsystems.hardware.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {
    Optional<Double> getTX(int targetID);

    Optional<Double> getTY(int targetID);

    PhotonTrackedTarget getBestTarget();

    Optional<Pose2d> getTargetPose(int targetID);
}
