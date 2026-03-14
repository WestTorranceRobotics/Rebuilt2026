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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.CustomUnits;

@Logged
public class IntakeIOReal extends SubsystemBase implements IntakeIO {
    private final SparkMax intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax hoodMotor = new SparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);

    private double actualRPM = 0;
    private boolean isIntakeOn = false;

    private final ArmFeedforward hoodFF = new ArmFeedforward(0, 0, 0);
    private final PIDController hoodPID = new PIDController(0, 0, 0);

    public IntakeIOReal() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodConfig.encoder.positionConversionFactor(2 * Math.PI); // Converts from rotations to radians
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hoodMotor.getEncoder().setPosition(Math.PI / 2);

        hoodPID.setTolerance(0.05);
        hoodPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public boolean isIntakeOn() {
        return isIntakeOn;
    }

    public AngularVelocity getIntakeSpeed() {
        return CustomUnits.RotationsPerMinute.of(actualRPM);
    }

    public void setIntakeVoltage(Voltage voltage) {
        intakeMotor.setVoltage(voltage);
        isIntakeOn = true;
    }

    public void stopIntake() {
        intakeMotor.set(0);
        isIntakeOn = false;
    }

    @Override
    public Command setHoodAngleCommand(Angle angle) {
        hoodPID.setSetpoint(angle.in(Radians));
        hoodPID.calculate(hoodMotor.getEncoder().getPosition());
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
    public Command stopHoodCommand() {
        return this.runOnce(() -> {
            hoodMotor.set(0);
        });
    }

    @Override
    public void periodic() {
        this.actualRPM = intakeMotor.getEncoder().getVelocity();
        SmartDashboard.putNumber("Intake RPM", actualRPM);
    }
}
