package frc.robot.subsystems.Shooter;


import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.ShooterConstants.*;
import frc.robot.utilities.CustomUnits;

public class ShooterIOSim extends SubsystemBase implements ShooterIO {
    private SparkMax feederMotor = new SparkMax(feederMotorID, MotorType.kBrushless);
    // TODO: implement feeder sim in periodic
    private SparkMaxSim feederMotorSim;

    private final SparkMax launcherMotorLeader = new SparkMax(firstIntakeMotorID, MotorType.kBrushless);
    private final SparkMax launcherMotorFollower = new SparkMax(secondIntakeMotorID, MotorType.kBrushless);
    private final SparkMax secondLauncherMotorFollower = new SparkMax(thirdIntakeMotorID, MotorType.kBrushless);

    private SparkMaxSim launcherMotorLeaderSim;
    private SparkMaxSim launcherMotorFollowerSim;
    private SparkMaxSim secondLauncherMotorFollowerSim;

    private FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(3), 0.00062156662, 1), DCMotor.getNEO(3)); // TODO update physical constants

    private final BangBangController bangbang = new BangBangController();
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.00235); // TODO: test for further tuning

    private double targetRPM = 0;
    private double actualRPM = 0;

    public ShooterIOSim() {
        // feeder config
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(feederMotorCurrentLimit);
        feederConfig.inverted(true);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // init feeder sim motor
        feederMotorSim = new SparkMaxSim(feederMotor, DCMotor.getNEO(1));

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

        // init launcher sim motors
        launcherMotorLeaderSim = new SparkMaxSim(launcherMotorLeader, DCMotor.getNEO(1));
        launcherMotorFollowerSim = new SparkMaxSim(launcherMotorFollower, DCMotor.getNEO(1));
        secondLauncherMotorFollowerSim = new SparkMaxSim(secondLauncherMotorFollower, DCMotor.getNEO(1));
    }

    public void setFlywheelSpeed(AngularVelocity velocity) {
        this.targetRPM = velocity.in(CustomUnits.RotationsPerMinute);
        bangbang.setSetpoint(targetRPM);
        double voltage = bangbang.calculate(
            launcherMotorLeader.getEncoder().getVelocity()) * RoboRioSim.getVInVoltage()
            + 0.9 * feedforward.calculate(targetRPM);
        
        launcherMotorLeader.setVoltage(voltage);
        launcherMotorFollower.setVoltage(voltage);
        secondLauncherMotorFollower.setVoltage(voltage);
    }

    public void setFlywheelVoltageDirectly(Voltage voltage) {
        launcherMotorLeader.set(voltage.in(Volts));
        launcherMotorFollower.set(voltage.in(Volts));
        secondLauncherMotorFollower.set(voltage.in(Volts));
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
        flywheelSim.setInput(launcherMotorLeaderSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(0.02);

        // Update motors
        launcherMotorLeaderSim.iterate(
                flywheelSim.getAngularVelocityRPM(),
                RoboRioSim.getVInVoltage(), 0.02);

        launcherMotorFollowerSim.iterate(
                flywheelSim.getAngularVelocityRPM(),
                RoboRioSim.getVInVoltage(), 0.02);

        secondLauncherMotorFollowerSim.iterate(
                flywheelSim.getAngularVelocityRPM(),
                RoboRioSim.getVInVoltage(), 0.02);

       this.actualRPM = flywheelSim.getAngularVelocityRPM();
       SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
       SmartDashboard.putNumber("Shooter RPM", actualRPM);

        // TODO does this carry between sims? seems like it does
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    }
}