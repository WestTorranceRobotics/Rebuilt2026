package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ShooterIO extends Subsystem {
    public void setFlywheelSpeed(AngularVelocity velocity);

    public void setFlywheelVoltageDirectly(Voltage voltage);

    public void setFeederVoltageDirectly(Voltage voltage);

    public void stopFlywheel();

    public void stopFeeder();
}
