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
    public void execute() {
        int hubAprilTagID = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue) ? 25 : 10;
        Double targetHubYaw = vision.getTX(hubAprilTagID).orElse(null);

        if (targetHubYaw == null) {
            if (NEUTRAL_ZONE_APRILTAG_IDS.contains(vision.getBestTarget().getFiducialId())) {
                shooter.setFlywheelSpeed(RotationsPerMinute.of(PASSING_SHOOTER_RPM));
            } else {
                shooter.setFlywheelSpeed(RotationsPerMinute.of(MINIMUM_SHOOTER_RPM));
            }
            swerveDrive.setAlignStatus(false, 0);
        } else {
            if (Math.abs(targetHubYaw) < MINIMUM_YAW_DISTANCE_TO_SHOOT) {
                SmartDashboard.putNumber("DISTANCE TO HUB (METERS)", vision.getDistance(hubAprilTagID));
                shooter.setFlywheelSpeed(RotationsPerMinute.of(vision.getDistance(hubAprilTagID)));
            }
            swerveDrive.setAlignStatus(true, targetHubYaw);
        }
        hopper.setHopperSpeed();
        shooter.setFeederSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        hopper.stopHopper();
    }
}
