package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.ShooterConstants.*;
import static frc.robot.utilities.CustomUnits.RotationsPerMinute;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ShooterIOReal extends SubsystemBase implements ShooterIO {
    protected final SparkMax feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

    protected final SparkMax flywheelMotor = new SparkMax(LAUNCHER_MOTOR_1_ID, MotorType.kBrushless);
    protected final SparkMax flywheelMotorInverted = new SparkMax(LAUNCHER_MOTOR_2_ID, MotorType.kBrushless);

    protected final BangBangController bangbang = new BangBangController();
    protected final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.00242); // TODO: tune further

    protected double targetRPM = 0;
    protected double actualRPM = 0;

    public ShooterIOReal() {
        // feeder config
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
        feederConfig.inverted(true);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // launcher motor configs
        SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        flywheelConfig.idleMode(IdleMode.kCoast);
        flywheelConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        flywheelConfig.inverted(false);
        flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelConfig.inverted(true);
        flywheelMotorInverted.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        bangbang.setTolerance(0.1);
    }

    public double getFeederRPM() {
        return feederMotor.getEncoder().getVelocity();
    }

    public double getShooterRPM() {
        return flywheelMotor.getEncoder().getVelocity();
    }

    public Command runShooterCommand(AngularVelocity velocity) {
        return this.runEnd(
                () -> {
                    this.setFlywheelSpeed(velocity);
                    this.setFeederVoltageDirectly(Volts.of(FEEDER_VOLTAGE));
                },
                this::stopShooter);
    }

    public void setFlywheelSpeed(AngularVelocity velocity) {
        this.targetRPM = velocity.in(RotationsPerMinute);
        bangbang.setSetpoint(targetRPM);
        double voltage = (bangbang.calculate(flywheelMotor.getEncoder().getVelocity()) * RoboRioDataJNI.getVInVoltage())
                + 0.9 * feedforward.calculate(targetRPM);

        flywheelMotor.setVoltage(voltage);
        flywheelMotorInverted.setVoltage(voltage);
    }

    public void setFlywheelVoltageDirectly(Voltage voltage) {
        flywheelMotor.setVoltage(voltage);
        flywheelMotorInverted.setVoltage(voltage);
    }

    public Command stopShooterCommand() {
        return this.runOnce(this::stopShooter);
    }

    public void stopShooter() {
        this.targetRPM = 0;
        bangbang.setSetpoint(0);
        flywheelMotor.stopMotor();
        flywheelMotorInverted.stopMotor();
        feederMotor.stopMotor();
    }

    public Command runFeederCommand() {
        return this.startEnd(
                () -> {
                    setFeederVoltageDirectly(Volts.of(FEEDER_VOLTAGE));
                },
                this::stopFeeder);
    }

    public void setFeederVoltageDirectly(Voltage voltage) {
        feederMotor.setVoltage(voltage.in(Volts));
    }

    public Command stopFeederCommand() {
        return this.runOnce(this::stopFeeder);
    }

    public void stopFeeder() {
        feederMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        this.actualRPM = flywheelMotor.getEncoder().getVelocity();
        SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter RPM", actualRPM);
    }
}
