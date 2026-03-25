package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.HopperConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.CustomUnits;

public class HopperIOReal extends SubsystemBase implements HopperIO {
    protected final SparkMax hopperMotor = new SparkMax(HOPPER_MOTOR_ID, MotorType.kBrushless);

    protected double actualRPM = 0;

    public HopperIOReal() {
        SparkMaxConfig hopperConfig = new SparkMaxConfig();
        hopperConfig.smartCurrentLimit(HOPPER_MOTOR_CURRENT_LIMIT);
        hopperConfig.idleMode(IdleMode.kCoast);
        hopperConfig.inverted(true);
        hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public AngularVelocity getRollerSpeed() {
        return CustomUnits.RotationsPerMinute.of(actualRPM);
    }

    public Command runHopper() {
        return this.runEnd(
                () -> {
                    this.setRollerVoltage(Volts.of(HOPPER_VOLTAGE));
                },
                this::stopRollers);
    }

    public void setRollerVoltage(Voltage voltage) {
        hopperMotor.setVoltage(voltage);
    }

    public void stopRollers() {
        hopperMotor.set(0);
    }

    @Override
    public void periodic() {
        this.actualRPM = hopperMotor.getEncoder().getVelocity();
        SmartDashboard.putNumber("Hopper RPM", actualRPM);
    }
}
