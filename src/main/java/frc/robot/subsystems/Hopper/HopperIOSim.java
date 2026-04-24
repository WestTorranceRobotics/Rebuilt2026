package frc.robot.subsystems.Hopper;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HopperIOSim extends HopperIOReal {
    private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), 0.00062156662, 1), DCMotor.getNEO(2));

    private final SparkMaxSim hopperMotorSim;

    private double actualRPM = 0;

    public HopperIOSim() {
        hopperMotorSim = new SparkMaxSim(hopperMotor, DCMotor.getNEO(1));
    }

    @Override
    public void periodic() {
        flywheelSim.setInput(hopperMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(0.02);

        // Update motor
        hopperMotorSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        this.actualRPM = flywheelSim.getAngularVelocityRPM();
        SmartDashboard.putNumber("Hopper RPM", actualRPM);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    }
}
