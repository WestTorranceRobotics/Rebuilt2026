package frc.robot.subsystems.Hopper;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface HopperIO extends Subsystem {
    public boolean areHopperRollersOn();

    public AngularVelocity getRollerSpeed();

    public void startRollers();

    public void setRollerVoltage(Voltage voltage);

    public void stopRollers();
}
