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
    protected final SparkMax hoodMotor = new SparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);

    protected boolean isIntakeOn = false;

    protected final double INTAKE_VOLTAGE = 11;
    protected final double OUTTAKE_VOLTAGE = -11;

    protected final double HOOD_DOWN_VOLTAGE = 5;
    protected final double HOOD_UP_VOLTAGE = -4;

    public IntakeIOReal() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.smartCurrentLimit(HOOD_MOTOR_CURRENT_LIMIT);
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodConfig.inverted(true);
        hoodConfig.encoder.positionConversionFactor(2 * Math.PI); // Converts from rotations to radians
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // We're going to manually start the robot with the intake hood on top of the robot.
        // I'm setting the convention that this starting position is 90 degrees (pi/2 radians).
        // Our target angle (when the intake is down, level with the floor) should be zero degrees,
        hoodMotor.getEncoder().setPosition(Math.PI / 2);
    }

    public boolean isIntakeOn() {
        return isIntakeOn;
    }

    public double getIntakeRPM() {
        return intakeMotor.getEncoder().getVelocity();
    }

    public double getHoodRPM() {
        return hoodMotor.getEncoder().getVelocity();
    }

    public Command intakeCommand() {
        return runIntakeAtVoltageCommand(Volts.of(INTAKE_VOLTAGE));
    }

    public Command outtakeCommand() {
        return runIntakeAtVoltageCommand(Volts.of(OUTTAKE_VOLTAGE));
    }

    public Command runIntakeAtVoltageCommand(Voltage voltage) {
        return this.startEnd(
                () -> {
                    intakeMotor.setVoltage(voltage);
                    isIntakeOn = true;
                },
               this::stopIntakeCommand
               );
    }

    public Command stopIntakeCommand() {
        return this.runOnce(() -> {
            intakeMotor.setVoltage(0);
            isIntakeOn = false;
        });
    }

    public Command sendHoodDownCommand() {
        return runHoodAtVoltageCommand(Volts.of(HOOD_DOWN_VOLTAGE));
    }

    public Command sendHoodUpCommand() {
        return runHoodAtVoltageCommand(Volts.of(HOOD_UP_VOLTAGE));
    }

    /**
     * Runs the hood motor at a given voltage. Positive voltage sends the intake DOWN, negative sends it UP.
     * @param voltage Voltage to be applied to the hood motor
     * @return Command that runs the hood motor at the given voltage
     */
    public Command runHoodAtVoltageCommand(Voltage voltage) {
        return this.startEnd(
                () -> {
                    hoodMotor.setVoltage(voltage);
                },
                this::stopHoodCommand);
    }

    public Command stopHoodCommand() {
        return this.runOnce(() -> {
            hoodMotor.setVoltage(0);
        });
    }
}
