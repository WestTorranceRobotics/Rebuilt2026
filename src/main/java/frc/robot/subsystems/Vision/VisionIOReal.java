package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionIOReal extends SubsystemBase implements VisionIO {
  PhotonCamera camera;
  List<PhotonTrackedTarget> trackedTargets;

  public void tick() {
    var results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        trackedTargets = result.getTargets();
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