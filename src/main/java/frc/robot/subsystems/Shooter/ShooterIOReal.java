package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIOReal extends SubsystemBase implements ShooterIO {

    @Override
    public AngularVelocity getFlywheelSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFlywheelSpeed'");
    }

    @Override
    public void setFlywheelSpeed(AngularVelocity velocity) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFlywheelSpeed'");
    }

    @Override
    public void setFlywheelVoltageDirectly(Voltage voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFlywheelVoltageDirectly'");
    }

    @Override
    public void stopFlywheel() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopFlywheel'");
    }

}