package frc.robot.subsystems.hardware.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {
    Optional<Double> getTX(int targetID);

    Optional<Double> getTY(int targetID);

    PhotonTrackedTarget getBestTarget();

    Optional<EstimatedRobotPose> getEstimatedRobotPose();

    Optional<Pose2d> getTargetPoseOfAprilTag(int targetID);

    Optional<Double> getDistance(int targetID);

    void tick();
}
