package frc.robot.commands.Shooter;

import static frc.robot.constants.ShooterConstants.*;
import static frc.robot.constants.VisionConstants.*;
import static frc.robot.utilities.CustomUnits.RotationsPerMinute;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper.HopperIO;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.subsystems.hardware.vision.VisionIO;

public class ShootCommand extends Command {
    private final ShooterIO shooter;
    private final SwerveDrive swerveDrive;
    private final VisionIO vision;
    private final HopperIO hopper;

    public ShootCommand(ShooterIO shooter, SwerveDrive swerveDrive, VisionIO vision, HopperIO hopper) {
        this.shooter = shooter;
        this.swerveDrive = swerveDrive;
        this.vision = vision;
        this.hopper = hopper;

        addRequirements(shooter, hopper);
    }

    @Override
    public void initialize() {
        hopper.runHopper();
        shooter.stopFeeder();
    }

    @Override
    public void execute() {
        if (shooter.shooterIsUpToSpeed()) {
            shooter.runFeeder();
        }

        int hubAprilTagID =
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Blue)
                        ? 25
                        : 10;

        var targetHubYaw = vision.getTX(hubAprilTagID);

        if (targetHubYaw.isEmpty()) {
            swerveDrive.setAlignStatus(false, 0);
            SmartDashboard.putNumber("Yaw from target", -1);

            if (vision.getBestTarget() != null
                    && NEUTRAL_ZONE_APRILTAG_IDS.contains(vision.getBestTarget().getFiducialId())) {
                // If robot is in the neutral zone / sees a neutral zone Apriltag
                shooter.setFlywheelSpeed(RotationsPerMinute.of(PASSING_SHOOTER_RPM));
            } else {
                shooter.setFlywheelSpeed(RotationsPerMinute.of(MINIMUM_SHOOTER_RPM));
            }
        } else {
            swerveDrive.setAlignStatus(true, targetHubYaw.get());
            SmartDashboard.putNumber("Yaw from target", targetHubYaw.get());
            SmartDashboard.putNumber("Distance from hub", vision.getDistance(hubAprilTagID));

            if (Math.abs(targetHubYaw.get()) <= YAW_ACCEPTABLE_ERROR) {
                shooter.setFlywheelSpeed(
                        RotationsPerMinute.of(DISTANCE_VS_RPM_MAP.get(vision.getDistance(hubAprilTagID))));
            } else {
                shooter.stopShooter();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.setAlignStatus(false, 0);
        shooter.stopShooter();
        hopper.stopHopper();
    }
}
