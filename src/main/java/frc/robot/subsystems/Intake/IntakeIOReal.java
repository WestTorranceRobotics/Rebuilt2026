package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeIOReal extends SubsystemBase implements IntakeIO {
    public SparkMax IntakeMotor = new SparkMax(0, MotorType.kBrushless);

    double speed = 0.4;

    @Override
    public boolean isIntakeOn() {
        return speed > 0;
    }

    @Override
    public double getIntakeSpeed() {
        return speed;
    
    }

    @Override
    public void startIntake() {
       IntakeMotor.set(0.4);

    }

    @Override
    public void setIntakeVoltage(Voltage voltage) {
     IntakeMotor.setVoltage(voltage);   
    }

    @Override
    public void stopIntake() {
      IntakeMotor.set(0);
    }

}