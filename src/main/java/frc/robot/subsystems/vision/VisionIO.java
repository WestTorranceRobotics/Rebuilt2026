package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {
    void tick();

    void updateInputs(VisionIOInputs inputs);

    public static class VisionIOInputs {
        public List<PhotonTrackedTarget> trackedTargets;
        public PhotonTrackedTarget bestTarget;
        public Optional<EstimatedRobotPose> estimatedPose;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
