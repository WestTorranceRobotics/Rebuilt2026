package frc.robot.subsystems.Climber;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ClimberIO extends Subsystem {
    public Distance getCurrentPosition();

    public LinearVelocity getCurrentVelocity();

    public Voltage getVoltage();

    public void setTargetPosition(Distance distance);

    public void stopClimber();
}