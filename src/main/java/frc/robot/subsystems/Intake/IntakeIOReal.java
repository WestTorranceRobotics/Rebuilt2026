package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.IntakeConstants.*;
import frc.robot.utilities.CustomUnits;

public class IntakeIOReal extends SubsystemBase implements IntakeIO {
    private final SparkMax intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);

    private double actualRPM = 0;
    private boolean isIntakeOn = false;

    public IntakeIOReal() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.smartCurrentLimit(intakeMotorCurrentLimit);
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    public void periodic() {
        this.actualRPM = intakeMotor.getEncoder().getVelocity();
        SmartDashboard.putNumber("Intake RPM", actualRPM);
    }
}