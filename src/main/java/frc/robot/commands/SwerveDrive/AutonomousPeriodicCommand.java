package frc.robot.commands.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

public class AutonomousPeriodicCommand extends Command {
    private final SwerveDrive swerveDrive;

    public AutonomousPeriodicCommand(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.tickPid();
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(new ChassisSpeeds());
        swerveDrive.tickPid();
    }
}
