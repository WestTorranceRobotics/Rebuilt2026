package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.IntakeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class IntakeIOReal extends SubsystemBase implements IntakeIO {
    protected final SparkMax intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    protected final SparkMax pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);

    public IntakeIOReal() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeConfig.inverted(true);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.smartCurrentLimit(PIVOT_MOTOR_CURRENT_LIMIT);
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.inverted(true);
        pivotConfig.encoder.positionConversionFactor(2 * Math.PI); // Converts from rotations to radians
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // We're going to manually start the robot with the intake pivot on top of the robot.
        // I'm setting the convention that this starting position is 90 degrees (pi/2 radians).
        // Our target angle (when the intake is down, level with the floor) should be zero degrees,
        pivotMotor.getEncoder().setPosition(Math.PI / 2);
    }

    public double getIntakeRPM() {
        return intakeMotor.getEncoder().getVelocity();
    }

    public double getHoodRPM() {
        return pivotMotor.getEncoder().getVelocity();
    }

    public Command intakeCommand() {
        return this.runIntakeAtVoltageCommand(Volts.of(INTAKE_VOLTAGE));
    }

    public Command outtakeCommand() {
        return this.runIntakeAtVoltageCommand(Volts.of(OUTTAKE_VOLTAGE));
    }

    public Command runIntakeAtVoltageCommand(Voltage voltage) {
        return this.startEnd(
                () -> {
                    this.setIntakeVoltage(voltage);
                },
                this::stopIntake);
    }

    public Command stopIntakeCommand() {
        return this.runOnce(() -> {
            this.stopIntake();
        });
    }

    public void outtake() {
        intakeMotor.setVoltage(OUTTAKE_VOLTAGE);
    }

    public void intake() {
        intakeMotor.setVoltage(INTAKE_VOLTAGE);
    }

    public void setIntakeVoltage(Voltage voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public void stopIntake() {
        intakeMotor.setVoltage(0);
    }

    public Command sendHoodDownCommand() {
        return this.runHoodAtVoltageCommand(Volts.of(PIVOT_DOWN_VOLTAGE));
    }

    public Command sendHoodUpCommand() {
        return this.runHoodAtVoltageCommand(Volts.of(PIVOT_UP_VOLTAGE));
    }

    /**
     * Runs the pivot motor at a given voltage. Positive voltage sends the intake DOWN, negative sends it UP.
     * @param voltage Voltage to be applied to the pivot motor
     * @return Command that runs the pivot motor at the given voltage
     */
    public Command runHoodAtVoltageCommand(Voltage voltage) {
        return this.runEnd(
                () -> {
                    runHoodAtVoltage(voltage);
                },
                this::stopHoodCommand);
    }

    public void runHoodAtVoltage(Voltage voltage) {
        pivotMotor.setVoltage(voltage);
    }

    public Command stopHoodCommand() {
        return this.runOnce(() -> {
            this.stopHood();
        });
    }

    public void stopHood() {
        pivotMotor.setVoltage(0);
    }
}
