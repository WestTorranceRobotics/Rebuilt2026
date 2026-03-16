package frc.robot.subsystems.Intake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

@Logged
public interface IntakeIO extends Subsystem {
    public boolean isIntakeOn();

    public AngularVelocity getIntakeSpeed();

    public void setIntakeVoltage(Voltage voltage);

    public void stopIntake();

    public void setHoodVoltage(Voltage voltage);

    public Command stopHoodCommand();
}
