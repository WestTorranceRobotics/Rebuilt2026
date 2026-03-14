package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.constants.IntakeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.CustomUnits;

@Logged
public class IntakeIOSim extends SubsystemBase implements IntakeIO {
	private final SparkMax intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
	private final SparkMaxSim intakeMotorSim;

	private final SparkMax hoodMotor = new SparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
	private final SparkMaxSim hoodMotorSim;

	private final ArmFeedforward hoodFF = new ArmFeedforward(0, 0, 0);
	private final PIDController hoodPID = new PIDController(0, 0, 0);

	// flywheel sim is being used because it's the closest to what we have
	private final FlywheelSim rollerSim = new FlywheelSim(
			LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.00062156662, 1),
			DCMotor.getNEO(1)); // TODO update physical constants

	private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
			// TODO: Find moment of inertia ("JKgSquaredMeters")
			LinearSystemId.createSingleJointedArmSystem(DCMotor.getNeo550(1), 11.8438079981694, 1),
			DCMotor.getNeo550(1),
			1.0 / 125,
			6,
			0,
			Math.PI / 2,
			true,
			Math.PI / 2
	);

	private double actualRPM = 0;
	private boolean isIntakeOn = false;

	public IntakeIOSim() {
		SparkMaxConfig intakeConfig = new SparkMaxConfig();
		intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
		intakeConfig.idleMode(IdleMode.kCoast);
		intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		intakeMotorSim = new SparkMaxSim(intakeMotor, DCMotor.getNEO(1));

		SparkMaxConfig hoodConfig = new SparkMaxConfig();
		hoodConfig.idleMode(IdleMode.kBrake);
		hoodConfig.encoder.positionConversionFactor(2 * Math.PI); // Converts from rotations to radians
		hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		hoodMotorSim = new SparkMaxSim(hoodMotor, DCMotor.getNEO(1));

		// We're going to manually start the robot with the intake hood on top of the robot.
		// I'm setting the convention that this starting position is 90 degrees (pi/2 radians).
		// Our target angle (when the intake is down, level with the floor) should be zero degrees,
		hoodMotor.getEncoder().setPosition(Math.PI / 2);

		hoodPID.setTolerance(0.05);
		hoodPID.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public boolean isIntakeOn() {
		return isIntakeOn;
	}

	@Override
	public AngularVelocity getIntakeSpeed() {
		return CustomUnits.RotationsPerMinute.of(actualRPM);
	}

	@Override
	public void setIntakeVoltage(Voltage voltage) {
		intakeMotor.setVoltage(voltage);
		isIntakeOn = true;
	}

	@Override
	public void stopIntake() {
		intakeMotor.set(0);
		isIntakeOn = false;
	}

	public Command setHoodAngleCommand(Angle angle) {
		hoodPID.setSetpoint(angle.in(Radians));
		hoodPID.calculate(hoodMotor.getEncoder().getPosition()); // Intentionally not used
		// ^ Run because .atSetpoint() doesn't return true until .calculate()'s been run at least once

		return this.run(() -> {
					hoodMotor.set(
							hoodFF.calculate(
									hoodMotor.getEncoder().getPosition(),
									hoodMotor.getEncoder().getVelocity())
							// + hoodPID.calculate(hoodMotor.getEncoder().getPosition())
					);
				})
				.until(hoodPID::atSetpoint)
				.andThen(this::stopHoodCommand);
	}

	public Command lowerHoodCommand() {
		return setHoodAngleCommand(Radians.of(0));
	}

	public Command raiseHoodCommand() {
		return setHoodAngleCommand(Radians.of(Math.PI / 2));
	}

	public Command zeroHoodAngleCommand() {
		return this.runOnce(() -> {
			hoodMotor.getEncoder().setPosition(0);
		});
	}

	public Command stopHoodCommand() {
		return this.runOnce(() -> {
			hoodMotor.set(0);
		});
	}

	@Override
	public void periodic() {
		rollerSim.setInput(intakeMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
		rollerSim.update(0.02);
		hoodSim.setInput(hoodMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
		hoodSim.update(0.02);

		// Update motors
		intakeMotorSim.iterate(rollerSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
		hoodMotorSim.iterate(hoodSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);

		this.actualRPM = rollerSim.getAngularVelocityRPM();
		SmartDashboard.putNumber("Intake RPM", actualRPM);

		// TODO does this carry between sims? seems like it does
		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
				rollerSim.getCurrentDrawAmps() + hoodSim.getCurrentDrawAmps()));
	}
}
