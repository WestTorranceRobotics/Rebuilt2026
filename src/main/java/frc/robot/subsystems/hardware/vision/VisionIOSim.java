package frc.robot.subsystems.hardware.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotContainer;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSim implements VisionIO {
    VisionSystemSim visionSystemSim = new VisionSystemSim("main");
    PhotonCamera camera;
    PhotonCameraSim cameraSim;

    EstimateConsumer estConsumer;

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, ROBOT_TO_CAM);
    Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

    List<PhotonTrackedTarget> trackedTargets;
    PhotonTrackedTarget bestTarget;

    public VisionIOSim(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;

        SimCameraProperties cameraProps = new SimCameraProperties();

        // Diagonal FOV was calculated from horizontal and vertical FOV given from
        // https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3#hardware-specifications
        cameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(74.34285844));
        cameraProps.setCalibError(0.25, 0.08);
        cameraProps.setFPS(50);
        cameraProps.setAvgLatencyMs(16);
        cameraProps.setLatencyStdDevMs(5);

        camera = new PhotonCamera(CAMERA_NAME);
        cameraSim = new PhotonCameraSim(camera, cameraProps);

        Rotation3d cameraRotation =
                new Rotation3d(0, Degrees.of(-15).in(Radians), Degrees.of(180).in(Radians));
        Transform3d cameraPosition = new Transform3d(new Translation3d(-0.3, 0, 0), cameraRotation);

        visionSystemSim.addCamera(cameraSim, cameraPosition);

        cameraSim.enableProcessedStream(true);
        cameraSim.enableRawStream(true);
        cameraSim.setTargetSortMode(PhotonTargetSortMode.Centermost);

        cameraSim.enableDrawWireframe(true);

        try {
            visionSystemSim.addAprilTags(
                    AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile));
        } catch (IOException ioe) {
            System.out.println("Failed to load april tag field");
        }
    }

    public void tick() {
        visionSystemSim.update(RobotContainer.swerveDriveSimulation.getSimulatedDriveTrainPose());

        var results = camera.getAllUnreadResults();

        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);

            if (result.hasTargets()) {
                trackedTargets = result.getTargets();
                bestTarget = result.getBestTarget();

                estimatedPose = photonEstimator.estimateCoprocMultiTagPose(result);
                if (estimatedPose.isEmpty()) estimatedPose = photonEstimator.estimateClosestToCameraHeightPose(result);

                estimatedPose.ifPresent(est -> {
                    // The heading is flipped to account for camera being on the "back"
                    int tagCount = est.targetsUsed.size();
                    double averageDistance = 0;
                    double maxAmbiguity = est.targetsUsed.stream()
                            .mapToDouble(target -> target.getPoseAmbiguity())
                            .max()
                            .orElse(1.0);
                    for (PhotonTrackedTarget target : est.targetsUsed) {
                        averageDistance +=
                                target.getBestCameraToTarget().getTranslation().getNorm();
                    }
                    averageDistance /= tagCount;

                    double xyStdDev = BASE_STD_DEV
                            * (1.0 + (averageDistance * DISTANCE_WEIGHT))
                            * (1.0 + (maxAmbiguity * AMBIGUITY_WEIGHT))
                            / Math.sqrt(tagCount);

                    estConsumer.accept(
                            est.estimatedPose
                                    .toPose2d()
                                    .transformBy(new Transform2d(new Translation2d(), Rotation2d.k180deg)),
                            est.timestampSeconds,
                            VecBuilder.fill(xyStdDev, xyStdDev, Double.MAX_VALUE));
                });
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
        PhotonTrackedTarget target = getTrackedTarget(targetID);
        if (target != null) return Optional.of(target.getYaw());
        return Optional.empty();
    }

    @Override
    public Optional<Double> getTY(int targetID) {
        PhotonTrackedTarget target = getTrackedTarget(targetID);
        if (target != null) return Optional.of(target.getPitch());
        return Optional.empty();
    }
}
