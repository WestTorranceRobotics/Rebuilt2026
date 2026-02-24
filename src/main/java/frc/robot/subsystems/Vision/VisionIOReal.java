package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;

import frc.robot.Constants.RealVisionIOConstants;

public class VisionIOReal implements VisionIO {
  public final PhotonCamera Camera;

  public VisionIOReal() {
    Camera = new PhotonCamera(RealVisionIOConstants.CameraName);
  }

  @Override
  public Optional<Double> getTX(int targetID) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTX'");
  }

  @Override
  public Optional<Double> getTY(int targetID) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTY'");
  }

}