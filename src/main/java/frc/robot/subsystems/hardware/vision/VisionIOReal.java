package frc.robot.subsystems.hardware.vision;

import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
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

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, ROBOT_TO_CAM);
    Optional<EstimatedRobotPose> estimatedPose;

    int targetID;

    public VisionIOReal() {
        camera = new PhotonCamera(CAMERA_NAME);
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }

    public void tick() {
        var results = camera.getAllUnreadResults();

        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                trackedTargets = result.getTargets();
                bestTarget = result.getBestTarget();

                estimatedPose = photonEstimator.estimateCoprocMultiTagPose(result);
            }
        }
    }

    private PhotonTrackedTarget getTrackedTarget(int targetID) {
        if (trackedTargets != null && !trackedTargets.isEmpty()) {
            for (PhotonTrackedTarget trackedTarget : trackedTargets) {
                if (trackedTarget.fiducialId == targetID) {
                    return trackedTarget;
                }
            }
        }
        return null;
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        if (estimatedPose.isPresent()) {
            return estimatedPose;
        } else {
            return Optional.empty();
        }
    }

    public Optional<Pose2d> getTargetPoseOfAprilTag(int targetID) {
        Pose2d pose = aprilTagFieldLayout.getTagPose(targetID).get().toPose2d();

        if (pose != null) return Optional.of(pose);
        return Optional.empty();
    }

    public Optional<Double> getDistance(int targetID) {
        if (getTrackedTarget(targetID) == null) return Optional.empty();
        return Optional.of(getTrackedTarget(targetID)
                .getBestCameraToTarget()
                .getTranslation()
                .getNorm());
    }

    public PhotonTrackedTarget getBestTarget() {
        return bestTarget;
    }

    @Override
    public Optional<Double> getTX(int targetID) {
        this.targetID = targetID;
        PhotonTrackedTarget target = getTrackedTarget(targetID);
        if (target != null) return Optional.of(target.getYaw());
        return Optional.empty();
    }

    @Override
    public Optional<Double> getTY(int targetID) {
        this.targetID = targetID;
        PhotonTrackedTarget target = getTrackedTarget(targetID);
        if (target != null) return Optional.of(target.getPitch());
        return Optional.empty();
    }
}
