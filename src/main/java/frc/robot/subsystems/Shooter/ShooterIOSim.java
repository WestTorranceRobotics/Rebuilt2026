package frc.robot.subsystems.Shooter;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOSim extends ShooterIOReal implements ShooterIO {
    // TODO: implement feeder sim in periodic
    private final SparkMaxSim feederMotorSim;

    private final SparkMaxSim launcherMotorLeaderSim;
    private final SparkMaxSim launcherMotorFollowerSim;

    private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(3), 0.00062156662, 1), DCMotor.getNEO(3));

    private final FlywheelSim feederSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.00062156662, 1), DCMotor.getNEO(1));

    public ShooterIOSim() {
        feederMotorSim = new SparkMaxSim(feederMotor, DCMotor.getNEO(1));

        launcherMotorLeaderSim = new SparkMaxSim(flywheelMotor, DCMotor.getNEO(1));
        launcherMotorFollowerSim = new SparkMaxSim(flywheelMotorInverted, DCMotor.getNEO(1));
    }

    @Override
    public void periodic() {
        flywheelSim.setInput(launcherMotorLeaderSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(0.02);

        feederSim.setInput(feederMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        feederSim.update(0.02);

        // Update motors
        feederMotorSim.iterate(feederSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
        launcherMotorLeaderSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
        launcherMotorFollowerSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        this.actualRPM = flywheelSim.getAngularVelocityRPM();
        SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter RPM", flywheelSim.getAngularVelocityRPM());
        SmartDashboard.putNumber("Feeder RPM", feederSim.getAngularVelocityRPM());

        // TODO does this carry between sims? seems like it does
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    }
}
