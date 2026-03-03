package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.ShooterConstants.*;
import frc.robot.utilities.CustomUnits;
import static edu.wpi.first.units.Units.*;

public class ShooterIOReal extends SubsystemBase implements ShooterIO {
    private final SparkMax feederMotor = new SparkMax(feederMotorID, MotorType.kBrushless);

    private final SparkMax launcherMotorLeader = new SparkMax(firstIntakeMotorID, MotorType.kBrushless);
    private final SparkMax launcherMotorFollower = new SparkMax(secondIntakeMotorID, MotorType.kBrushless);
    private final SparkMax secondLauncherMotorFollower = new SparkMax(thirdIntakeMotorID, MotorType.kBrushless);

    private final BangBangController bangbang = new BangBangController();
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.00255); // TODO: test for further tuning

    private double targetRPM = 0;
    private double actualRPM = 0;

    public ShooterIOReal() {
        // feeder config
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(feederMotorCurrentLimit);
        feederConfig.inverted(true);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // launcher motor configs
        SparkMaxConfig launcherLeaderConfig = new SparkMaxConfig();
        launcherLeaderConfig.idleMode(IdleMode.kCoast);
        launcherLeaderConfig.smartCurrentLimit(launcherMotorCurrentLimit);
        launcherLeaderConfig.inverted(false);
        launcherMotorLeader.configure(launcherLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig launcherFollowerConfig = new SparkMaxConfig();
        launcherFollowerConfig.idleMode(IdleMode.kCoast);
        launcherFollowerConfig.smartCurrentLimit(launcherMotorCurrentLimit);
        launcherFollowerConfig.inverted(true);
        launcherMotorFollower.configure(launcherFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig secondLauncherFollowerConfig = new SparkMaxConfig();
        secondLauncherFollowerConfig.idleMode(IdleMode.kCoast);
        secondLauncherFollowerConfig.smartCurrentLimit(launcherMotorCurrentLimit);
        secondLauncherFollowerConfig.inverted(true); // FIXME find correct inversion
        secondLauncherMotorFollower.configure(secondLauncherFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // if inversion types are same between the follower configs, it may be cleaner to reuse the config for both of them
    }    

    public void setFlywheelSpeed(AngularVelocity velocity) {
        this.targetRPM = velocity.in(CustomUnits.RotationsPerMinute);
        bangbang.setSetpoint(targetRPM);
        double voltage = bangbang.calculate(
            launcherMotorLeader.getEncoder().getVelocity()) * RoboRioDataJNI.getVInVoltage()
            + 0.9 * feedforward.calculate(targetRPM);
        
        launcherMotorLeader.setVoltage(voltage);
        launcherMotorFollower.setVoltage(voltage);
        secondLauncherMotorFollower.setVoltage(voltage);
    }

    public void setFlywheelVoltageDirectly(Voltage voltage) {
        launcherMotorLeader.set(voltage.in(Volts));
        launcherMotorFollower.set(voltage.in(Volts));
        secondLauncherMotorFollower.set(voltage.in(Volts));
    }

    public void stopFlywheel() {
        launcherMotorLeader.set(0);
        launcherMotorFollower.set(0);
        secondLauncherMotorFollower.set(0);
        targetRPM = 0;
    }

    public void setFeederVoltageDirectly(Voltage voltage) {
        feederMotor.set(voltage.in(Volts));
    }

    public void stopFeeder() {
        feederMotor.set(0);
    }

    @Override
    public void periodic() {
        this.actualRPM = launcherMotorLeader.getEncoder().getVelocity();
        SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter RPM", actualRPM);
    }
}