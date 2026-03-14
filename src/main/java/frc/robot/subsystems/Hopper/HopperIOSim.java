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
    private final SparkMax hopperMotorLeader = new SparkMax(HOPPER_MOTOR_LEADER_ID, MotorType.kBrushless);
    private final SparkMax hopperMotorFollower = new SparkMax(HOPPER_MOTOR_FOLLOWER_ID, MotorType.kBrushless);

    // flywheel sim is being used because it's the closest to what we have
    private FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), 0.00062156662, 1),
            DCMotor.getNEO(2)); // TODO update physical constants

    private final SparkMaxSim hopperMotorLeaderSim;
    private final SparkMaxSim hopperMotorFollowerSim;

    private double actualRPM = 0;
    private boolean isHopperOn = false;

    public HopperIOSim() {
        SparkMaxConfig hopperLeaderConfig = new SparkMaxConfig();
        hopperLeaderConfig.smartCurrentLimit(HOPPER_MOTOR_CURRENT_LIMIT);
        hopperLeaderConfig.idleMode(IdleMode.kCoast);
        hopperMotorLeader.configure(hopperLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig hopperFollowerConfig = new SparkMaxConfig();
        hopperFollowerConfig.smartCurrentLimit(HOPPER_MOTOR_CURRENT_LIMIT);
        hopperFollowerConfig.idleMode(IdleMode.kCoast);
        hopperFollowerConfig.follow(hopperMotorLeader); // TODO check if following works
        hopperMotorFollower.configure(
                hopperFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hopperMotorLeaderSim = new SparkMaxSim(hopperMotorLeader, DCMotor.getNEO(1));
        hopperMotorFollowerSim = new SparkMaxSim(hopperMotorFollower, DCMotor.getNEO(1));
    }

    public boolean areHopperRollersOn() {
        return isHopperOn;
    }

    public AngularVelocity getRollerSpeed() {
        return CustomUnits.RotationsPerMinute.of(actualRPM);
    }

    public void setRollerVoltage(Voltage voltage) {
        hopperMotorLeader.setVoltage(voltage);
        isHopperOn = true;
    }

    public void stopRollers() {
        hopperMotorLeader.setVoltage(0);
        isHopperOn = false;
    }

    @Override
    public void periodic() {
        flywheelSim.setInput(hopperMotorLeaderSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(0.02);

        // Update motors
        hopperMotorLeaderSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        hopperMotorFollowerSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        this.actualRPM = flywheelSim.getAngularVelocityRPM();
        SmartDashboard.putNumber("Rollers RPM", actualRPM);

        // TODO does this carry between sims? seems like it does
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    }
}
