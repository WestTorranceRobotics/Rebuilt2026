package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionIOReal extends SubsystemBase implements VisionIO {
  PhotonCamera camera;
  List<PhotonTrackedTarget> trackedTargets;

  public VisionIOReal() {
    SimCameraProperties cameraProps = new SimCameraProperties();

    // Diagonal FOV was calculated from horizontal and vertical FOV given from
    // https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3#hardware-specifications
    cameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(74.34285844));
    cameraProps.setCalibError(0.25, 0.08);
    cameraProps.setFPS(50);
    cameraProps.setAvgLatencyMs(16);
    cameraProps.setLatencyStdDevMs(5);

    camera = new PhotonCamera(Constants.VisionIOConstants.CameraName);

    Rotation3d cameraRotation = new Rotation3d(0, Radians.of(Degrees.of(-15).in(Radian)).magnitude(), 0);
    Transform3d cameraPosition = new Transform3d(Translation3d.kZero, cameraRotation);
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
  
  @Override
  public void periodic() {
    var results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        trackedTargets = result.getTargets();
      }
    }
  }

  @Override
  public Optional<Double> getTX(int targetID) {
    PhotonTrackedTarget target = getTrackedTarget(targetID);
    if (target != null)
      return Optional.of(target.getYaw());
    return Optional.empty();
  }

  @Override
  public Optional<Double> getTY(int targetID) {
    PhotonTrackedTarget target = getTrackedTarget(targetID);
    if (target != null)
      return Optional.of(target.getPitch());
    return Optional.empty();
  }
}