package frc.robot.subsystems.Intake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

@Logged
public interface IntakeIO extends Subsystem {
    public double getIntakeRPM();

    public double getHoodRPM();

    public Command intakeCommand();

    public Command outtakeCommand();

    public Command stopIntakeCommand();

    public Command runHoodAtVoltageCommand(Voltage voltage);

    public Command sendHoodUpCommand();

    public Command sendHoodDownCommand();

    public Command stopHoodCommand();

    public void runHoodAtVoltage(Voltage voltage);

    public void intake();

    public void outtake();

    public void stopIntake();

    public String getIntakeLocation();
}
