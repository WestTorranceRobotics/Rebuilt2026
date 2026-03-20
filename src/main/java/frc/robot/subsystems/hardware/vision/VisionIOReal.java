package frc.robot.subsystems.hardware.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOReal implements VisionIO {
    PhotonCamera camera;
    List<PhotonTrackedTarget> trackedTargets;
    PhotonTrackedTarget bestTarget;

    AprilTagFieldLayout aprilTagFieldLayout;

    int targetID;

    public VisionIOReal() {
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    }

    public void tick() {
        var results = camera.getAllUnreadResults();

        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                trackedTargets = result.getTargets();
                bestTarget = result.getBestTarget();
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

    public Optional<Pose2d> getTargetPose(int targetID) {
        Pose2d pose = aprilTagFieldLayout.getTagPose(targetID).get().toPose2d();

        if (pose != null) return Optional.of(pose);
        return Optional.empty();
    }

    public Double getDistance(int targetID) {
        return getTrackedTarget(targetID)
                .getBestCameraToTarget()
                .getTranslation()
                .getNorm();
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
