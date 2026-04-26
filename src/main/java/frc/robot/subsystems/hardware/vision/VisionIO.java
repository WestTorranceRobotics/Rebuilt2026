package frc.robot.subsystems.hardware.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
