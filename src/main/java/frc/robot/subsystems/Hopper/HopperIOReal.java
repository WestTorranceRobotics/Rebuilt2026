package frc.robot.subsystems.Hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.HopperConstants.*;
import frc.robot.utilities.CustomUnits;

public class HopperIOReal extends SubsystemBase implements HopperIO {
    private final SparkMax hopperMotorLeader = new SparkMax(hopperMotorLeaderID, MotorType.kBrushless);
    private final SparkMax hopperMotorFollower = new SparkMax(hopperMotorFollowerID, MotorType.kBrushless);

    private double actualRPM = 0;
    private boolean isHopperOn = false;

    public HopperIOReal() {
        SparkMaxConfig hopperLeaderConfig = new SparkMaxConfig();
        hopperLeaderConfig.smartCurrentLimit(hopperMotorCurrentLimit);
        hopperLeaderConfig.idleMode(IdleMode.kCoast);
        hopperMotorLeader.configure(hopperLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig hopperFollowerConfig = new SparkMaxConfig();
        hopperFollowerConfig.smartCurrentLimit(hopperMotorCurrentLimit);
        hopperFollowerConfig.idleMode(IdleMode.kCoast);
        hopperFollowerConfig.follow(hopperMotorLeader); // TODO check if following works
        hopperMotorFollower.configure(hopperFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        hopperMotorLeader.set(0);
        isHopperOn = false;
    }

    @Override
    public void periodic() {
        this.actualRPM = hopperMotorLeader.getEncoder().getVelocity();
        SmartDashboard.putNumber("Rollers RPM", actualRPM);
    }
}