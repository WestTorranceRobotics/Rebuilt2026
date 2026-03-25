package frc.robot.subsystems.Hopper;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface HopperIO extends Subsystem {
    public AngularVelocity getRollerSpeed();

    public Command runHopperCommand();

    public void setHopperSpeed();

    public void stopHopper();
}
