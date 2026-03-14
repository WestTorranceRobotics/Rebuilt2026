package frc.robot.subsystems.Hopper;

import static frc.robot.constants.HopperConstants.*;

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

public class HopperIOSim extends SubsystemBase implements HopperIO {
    private final SparkMax hopperMotor = new SparkMax(HOPPER_MOTOR_ID, MotorType.kBrushless);

    // flywheel sim is being used because it's the closest to what we have
    private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), 0.00062156662, 1),
            DCMotor.getNEO(2)); // TODO update physical constants

    private final SparkMaxSim hopperMotorSim;

    private double actualRPM = 0;
    private boolean isHopperOn = false;

    public HopperIOSim() {
        SparkMaxConfig hopperConfig = new SparkMaxConfig();
        hopperConfig.smartCurrentLimit(HOPPER_MOTOR_CURRENT_LIMIT);
        hopperConfig.idleMode(IdleMode.kCoast);
        hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hopperMotorSim = new SparkMaxSim(hopperMotor, DCMotor.getNEO(1));
    }

    public boolean areHopperRollersOn() {
        return isHopperOn;
    }

    public AngularVelocity getRollerSpeed() {
        return CustomUnits.RotationsPerMinute.of(actualRPM);
    }

    public void setRollerVoltage(Voltage voltage) {
        hopperMotor.setVoltage(voltage);
        isHopperOn = true;
    }

    public void stopRollers() {
        hopperMotor.setVoltage(0);
        isHopperOn = false;
    }

    @Override
    public void periodic() {
        flywheelSim.setInput(hopperMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(0.02);

        // Update motor
        hopperMotorSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        hopperMotorSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        this.actualRPM = flywheelSim.getAngularVelocityRPM();
        SmartDashboard.putNumber("Hopper RPM", actualRPM);

        // TODO does this carry between sims? seems like it does
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    }
}
