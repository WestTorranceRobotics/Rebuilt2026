package frc.robot.subsystems.Shooter;

import static frc.robot.constants.ShooterConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOSim extends ShooterIOReal implements ShooterIO {
    // TODO: implement feeder sim in periodic
    private SparkMaxSim feederMotorSim;

    private final SparkMaxSim launcherMotorLeaderSim;
    private final SparkMaxSim launcherMotorFollowerSim;

    private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(3), 0.00062156662, 1),
            DCMotor.getNEO(3)); // TODO update physical constants

    private final BangBangController bangbang = new BangBangController();
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.00235); // TODO: tune further

    private double targetRPM = 0;
    private double actualRPM = 0;

    public ShooterIOSim() {
        launcherMotorLeaderSim = new SparkMaxSim(flywheelMotor, DCMotor.getNEO(1));
        launcherMotorFollowerSim = new SparkMaxSim(flywheelMotorInverted, DCMotor.getNEO(1));
    }

    @Override
    public void periodic() {
        flywheelSim.setInput(launcherMotorLeaderSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(0.02);

        // Update motors
        launcherMotorLeaderSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
        launcherMotorFollowerSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        this.actualRPM = flywheelSim.getAngularVelocityRPM();
        SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter RPM", actualRPM);

        // TODO does this carry between sims? seems like it does
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    }
}
