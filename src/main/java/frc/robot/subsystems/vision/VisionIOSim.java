package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

    PhotonPoseEstimator photonEstimator =
            new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), ROBOT_TO_CAM);
    Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

    List<PhotonTrackedTarget> trackedTargets;
    PhotonTrackedTarget bestTarget;

    public VisionIOSim() {
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

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        tick();

        inputs.trackedTargets = trackedTargets;
        inputs.bestTarget = bestTarget;
        inputs.estimatedPose = estimatedPose;
    }

    @Override
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
            }
        }
    }
}
