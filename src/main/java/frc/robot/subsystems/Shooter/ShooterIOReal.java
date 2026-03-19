package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.CustomUnits;

public class ShooterIOReal extends SubsystemBase implements ShooterIO {
    private final SparkMax feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

    private final SparkMax launcherMotorLeader = new SparkMax(LAUNCHER_MOTOR_1_ID, MotorType.kBrushless);
    private final SparkMax launcherMotorFollower = new SparkMax(LAUNCHER_MOTOR_2_ID, MotorType.kBrushless);

    private final BangBangController bangbang = new BangBangController();
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.00242); // TODO: tune further

    private double targetRPM = 0;
    private double actualRPM = 0;

    public ShooterIOReal() {
        // feeder config
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
        feederConfig.inverted(true);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // launcher motor configs
        SparkMaxConfig launcherLeaderConfig = new SparkMaxConfig();
        launcherLeaderConfig.idleMode(IdleMode.kCoast);
        launcherLeaderConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        launcherLeaderConfig.inverted(false);
        launcherMotorLeader.configure(
                launcherLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig launcherFollowerConfig = new SparkMaxConfig();
        launcherFollowerConfig.idleMode(IdleMode.kCoast);
        launcherFollowerConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        launcherFollowerConfig.inverted(true);
        launcherMotorFollower.configure(
                launcherFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setFlywheelSpeed(AngularVelocity velocity) {
        this.targetRPM = velocity.in(CustomUnits.RotationsPerMinute);
        bangbang.setSetpoint(targetRPM);
        double voltage =
                bangbang.calculate(launcherMotorLeader.getEncoder().getVelocity()) * RoboRioDataJNI.getVInVoltage()
                        + 0.9 * feedforward.calculate(targetRPM);

        launcherMotorLeader.setVoltage(voltage);
        launcherMotorFollower.setVoltage(voltage);
    }

    public Command setFlywheelSpeedCommand(AngularVelocity velocity) {
        return this.run(
                () -> { // Commands.run runs repeatedly until interrupted.
                    setFlywheelSpeed(velocity);
                });
    }

    public void setFlywheelVoltageDirectly(Voltage voltage) {
        launcherMotorLeader.set(voltage.in(Volts));
        launcherMotorFollower.set(voltage.in(Volts));
    }

    public Command setFlywheelVoltageDirectlyCommand(Voltage voltage) {
        return this.runOnce(() -> {
            setFlywheelVoltageDirectly(voltage);
        });
    }

    public void stopShooter() {
        launcherMotorLeader.setVoltage(0);
        launcherMotorFollower.setVoltage(0);
        feederMotor.set(0);
        targetRPM = 0;
    }

    public void setFeederVoltageDirectly(Voltage voltage) {
        if (Math.abs(actualRPM - targetRPM) < 100) feederMotor.set(voltage.in(Volts));
    }

    @Override
    public void periodic() {
        this.actualRPM = launcherMotorLeader.getEncoder().getVelocity();
        SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter RPM", actualRPM);
    }
}
