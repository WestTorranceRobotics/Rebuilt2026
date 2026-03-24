package frc.robot.subsystems.Intake;

import static frc.robot.constants.IntakeConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

@Logged
public class IntakeIOSim extends IntakeIOReal implements IntakeIO {
    private final SparkMaxSim intakeMotorSim;
    private final SparkMaxSim pivotMotorSim;

    // flywheel sim is being used because it's the closest to what we have
    private final FlywheelSim rollerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.00062156662, 1),
            DCMotor.getNEO(1)); // TODO update physical constants

    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
            // TODO: Find moment of inertia ("JKgSquaredMeters")
            LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), 11.8438079981694, 1),
            DCMotor.getNEO(1),
            1.0 / 125,
            6,
            0,
            Math.PI / 2,
            true,
            Math.PI / 2);

    public IntakeIOSim() {
        intakeMotorSim = new SparkMaxSim(intakeMotor, DCMotor.getNEO(1));
        pivotMotorSim = new SparkMaxSim(pivotMotor, DCMotor.getNEO(1));
    }

    @Override
    public void periodic() {
        rollerSim.setInput(intakeMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        rollerSim.update(0.02);
        pivotSim.setInput(pivotMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        pivotSim.update(0.02);

        // Update motors
        intakeMotorSim.iterate(rollerSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
        pivotMotorSim.iterate(pivotSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);

        // TODO does this carry between sims? seems like it does
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                rollerSim.getCurrentDrawAmps() + pivotSim.getCurrentDrawAmps()));
    }
}
