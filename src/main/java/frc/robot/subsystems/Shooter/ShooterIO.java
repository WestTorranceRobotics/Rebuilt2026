package frc.robot.subsystems.Shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

@Logged
public interface ShooterIO extends Subsystem {
    public Command runShooterCommand(AngularVelocity velocity);
}
