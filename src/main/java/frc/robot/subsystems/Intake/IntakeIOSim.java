package frc.robot.subsystems.Intake;

import static frc.robot.constants.IntakeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.CustomUnits;

public class IntakeIOSim extends SubsystemBase implements IntakeIO {
    private final SparkMax intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
    private final SparkMaxSim intakeMotorSim;

    // flywheel sim is being used because it's the closest to what we have
    private FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.00062156662, 1),
            DCMotor.getNEO(1)); // TODO update physical constants

    private double actualRPM = 0;
    private boolean isIntakeOn = false;

    public IntakeIOSim() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.smartCurrentLimit(intakeMotorCurrentLimit);
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeMotorSim = new SparkMaxSim(intakeMotor, DCMotor.getNEO(1));
    }

    public boolean isIntakeOn() {
        return isIntakeOn;
    }

    public AngularVelocity getIntakeSpeed() {
        return CustomUnits.RotationsPerMinute.of(actualRPM);
    }

    public void setIntakeVoltage(Voltage voltage) {
        intakeMotor.setVoltage(voltage);
        isIntakeOn = true;
    }

    public void stopIntake() {
        intakeMotor.set(0);
        isIntakeOn = false;
    }

    @Override
    public void periodic() {
        flywheelSim.setInput(intakeMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(0.02);

        // Update motor
        intakeMotorSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        this.actualRPM = flywheelSim.getAngularVelocityRPM();
        SmartDashboard.putNumber("Intake RPM", actualRPM);

        // TODO does this carry between sims? seems like it does
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    }
}
