package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOReal implements VisionIO {
    PhotonCamera camera;
    List<PhotonTrackedTarget> trackedTargets;
    PhotonTrackedTarget bestTarget;

    PhotonPoseEstimator photonEstimator =
            new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), ROBOT_TO_CAM);
    Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

    public VisionIOReal() {
        camera = new PhotonCamera(CAMERA_NAME);
    }

    @Override
    public void tick() {
        var results = camera.getAllUnreadResults();

        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                trackedTargets = result.getTargets();
                bestTarget = result.getBestTarget();

                estimatedPose = photonEstimator.estimateCoprocMultiTagPose(result);
                if (estimatedPose.isEmpty()) estimatedPose = photonEstimator.estimateClosestToCameraHeightPose(result);
            }
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        tick();

        inputs.trackedTargets = trackedTargets;
        inputs.bestTarget = bestTarget;
        inputs.estimatedPose = estimatedPose;
    }
}
